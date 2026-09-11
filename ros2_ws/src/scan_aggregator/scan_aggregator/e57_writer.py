"""Minimal ASTM E57 writer, plus a PCD reader to feed it.

No e57/libE57Format dependency is installed (or installable, on the
Pi's aarch64 RoboStack conda env -- it's a compiled C++ library with no
aarch64 wheels on PyPI) in the ROS2 env this runs in, same reasoning
pcd_writer.py already gives for hand-rolling PCD instead of pulling in a
library for one format. E57 is a real binary format though (a paged,
CRC-32C-protected physical file structure wrapping an XML metadata
section and one or more packed binary point sections) -- "read the spec
and hope" was a real correctness risk for something this intricate with
no way to validate the result here. Instead this was reverse-engineered
against a real reference implementation (pye57/libE57Format, installed
here on the dev machine ONLY as a throwaway validation oracle -- never a
runtime dependency of this module or anything that ships to the Pi):
generated known-content reference .e57 files with it, parsed them
byte-by-byte to confirm the physical/logical paging model, the CRC
algorithm (CRC-32C, reflected, stored big-endian -- confirmed against
every page of three independently-generated reference files, not just
one), and the CompressedVector binary section's packet layout, then
validated this module's own output by opening it back up with that same
reference implementation. See HANDOFF.md for the write-up.

Physical file layout (all of this confirmed empirically, not just read
from the spec):
- A 48-byte header (fileSignature "ASTM-E57", majorVersion, minorVersion,
  filePhysicalLength, xmlPhysicalOffset, xmlLogicalLength, pageSize --
  all little-endian), NOT itself CRC-protected.
- Everything after that -- the CompressedVector binary section(s) then
  the XML text -- is one continuous logical byte stream that gets
  physically chunked into PAGE_SIZE-byte pages, each page being
  PAGE_PAYLOAD content bytes followed by a 4-byte big-endian CRC-32C of
  those bytes. Each page's CRC is independent (not chained across
  pages), which is what makes the vectorized _crc32c_pages below
  possible -- confirmed by validating every page of multiple reference
  files independently.
- xmlPhysicalOffset/xmlLogicalLength, and the CompressedVectorSection's
  own dataPhysicalOffset/sectionLogicalLength, are all offsets/lengths
  into that same logical (CRC-stripped) stream, despite "Physical" in
  some of the field names -- confirmed by successfully parsing real
  reference files' XML and point sections using exactly that
  interpretation.
- The file's total physical length is always an exact multiple of
  PAGE_SIZE -- the logical content is zero-padded up to the next page
  boundary before the physical file ends (confirmed against three
  reference files of different, non-page-aligned content lengths, all
  of which still landed on an exact page boundary).

CompressedVector binary section (this project only ever writes one,
holding all points for a single scan):
- A 32-byte section header: sectionId (uint8, 1), 7 reserved zero bytes,
  sectionLogicalLength (uint64), dataPhysicalOffset (uint64),
  indexPhysicalOffset (uint64, 0 -- no index is written).
- One or more "data packets" (the uint16 packet-length field caps each
  packet at 65536 logical bytes, so a real multi-million-point scan
  needs many packets, not one) -- each: packetType (uint8, 1),
  packetFlags (uint8, 0), packetLogicalLengthMinus1 (uint16),
  bytestreamCount (uint16), then bytestreamCount uint16 byte-lengths,
  then that many separate field byte-streams back to back in prototype
  order (columnar/struct-of-arrays, NOT interleaved per-point --
  confirmed by decoding real per-point values out of a reference file
  and finding all-X values, then all-Y, then all-Z, then all-intensity),
  padded with zero bytes to a multiple of 4.

Only the "Float" record type is used here (precision="single", i.e. a
plain IEEE754 float32 per value, uncompressed/unpacked) -- not the
scaled-integer bitpacked encoding E57 also supports, which needs no
codec entry either way for a reader to decode correctly, and matches
this project's data exactly: every point already arrives here as
float32 (see pcd_writer.py / _transform_and_accumulate's own
structured_to_unstructured(..., dtype=np.float32)), so this is a
straight, lossless re-encoding, not a new precision decision.
"""

from __future__ import annotations

import os
import struct
import time
import uuid
from typing import Optional

import numpy as np

PAGE_SIZE = 1024
PAGE_PAYLOAD = PAGE_SIZE - 4
# uint16 packetLogicalLengthMinus1 field caps a single data packet's
# logical length at 65536 bytes -- stay comfortably under that so the
# padding-to-multiple-of-4 step can never push a packet over the edge.
MAX_PACKET_LOGICAL_BYTES = 65000

_FIELD_NAMES = ("cartesianX", "cartesianY", "cartesianZ", "intensity")


def _crc32c_table() -> np.ndarray:
    poly = 0x82F63B78  # reflected CRC-32C (Castagnoli) polynomial
    table = np.empty(256, dtype=np.uint64)
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ poly if (crc & 1) else (crc >> 1)
        table[i] = crc
    return table


_CRC32C_TABLE = _crc32c_table()


def _crc32c_pages(payloads: np.ndarray) -> np.ndarray:
    """payloads: (num_pages, PAGE_PAYLOAD) uint8. Returns each page's
    CRC-32C as a uint32 array (still held as uint64 here to leave
    headroom for the intermediate XOR/shift steps -- narrowed by the
    caller when packing bytes).

    Vectorized across pages, not a per-byte Python loop over the whole
    file: since each page's CRC is independent of every other page (no
    chaining), every page can be folded through the table in lockstep,
    one shared byte-COLUMN at a time, across all pages at once. That
    turns an O(total file bytes) pure-Python loop -- genuinely too slow
    for a real multi-million-point scan's worth of pages -- into
    O(PAGE_PAYLOAD) numpy-vectorized steps regardless of file size.
    """
    crc = np.full(payloads.shape[0], 0xFFFFFFFF, dtype=np.uint64)
    for col in range(payloads.shape[1]):
        idx = (crc ^ payloads[:, col].astype(np.uint64)) & 0xFF
        crc = _CRC32C_TABLE[idx] ^ (crc >> 8)
    return (crc ^ 0xFFFFFFFF) & 0xFFFFFFFF


def _logical_to_physical_offset(logical_offset: int) -> int:
    """Converts a position measured in real content bytes (as if pages
    had no CRC footers at all) into the true raw byte position in the
    physical file, by adding back the 4 CRC bytes for every complete
    PAGE_PAYLOAD-byte chunk of content that precedes it.

    This matters for any field actually named "...Physical..." in the
    format (xmlPhysicalOffset in the file header; dataPhysicalOffset in
    a CompressedVectorSectionHeader) -- these are real seek targets a
    reader jumps straight to, unlike a packet's own
    packetLogicalLengthMinus1 (used only to walk sequentially from one
    already-located packet to the next, which stays purely logical/
    content-relative and needs no such conversion).

    Found the hard way, not derived correctly from memory of the spec
    the first time: an earlier version of this module stored the plain
    logical content count directly in xmlPhysicalOffset, which happened
    to equal the correct physical value for tiny test files (nothing
    before the XML section had crossed a page boundary yet, making the
    conversion a no-op) but produced a file real E57 readers could not
    open at all once real data pushed the XML section past the first
    page boundary -- caught by validating against pye57/libE57Format
    (installed as a throwaway dev-only oracle, see module docstring),
    not by re-reading the spec text alone. Confirmed against the exact
    stored values in multiple independently-generated reference files
    (found by literally searching a reference file's raw bytes for the
    `<?xml` prolog and comparing that true physical position against
    both the stored header field and this formula) before trusting it."""
    return logical_offset + 4 * (logical_offset // PAGE_PAYLOAD)


def _pad_to_page_boundary(content: bytes) -> bytes:
    remainder = len(content) % PAGE_PAYLOAD
    if remainder == 0:
        return content
    return content + b"\x00" * (PAGE_PAYLOAD - remainder)


def _physical_bytes_from_logical(content: bytes) -> bytes:
    """Chunks a logical (CRC-free) byte stream into PAGE_SIZE physical
    pages, appending each page's big-endian CRC-32C footer. `content`
    must already be padded to an exact multiple of PAGE_PAYLOAD (see
    _pad_to_page_boundary) -- every page here is a full page, matching
    what every real reference file examined during development did
    (see module docstring)."""
    n_pages = len(content) // PAGE_PAYLOAD
    payloads = np.frombuffer(content, dtype=np.uint8).reshape(n_pages, PAGE_PAYLOAD)
    crcs = _crc32c_pages(payloads).astype(np.uint32)
    out = bytearray(n_pages * PAGE_SIZE)
    for i in range(n_pages):
        start = i * PAGE_SIZE
        out[start:start + PAGE_PAYLOAD] = payloads[i].tobytes()
        struct.pack_into(">I", out, start + PAGE_PAYLOAD, int(crcs[i]))
    return bytes(out)


def _build_data_packets(columns: list[np.ndarray]) -> bytes:
    """columns: one float32 array per field (all same length N), in
    prototype order. Splits N records across as many data packets as
    needed to stay under MAX_PACKET_LOGICAL_BYTES, each packet holding a
    contiguous slice of records, each field's slice written as its own
    contiguous byte-stream (struct-of-arrays within the packet, see
    module docstring)."""
    n = columns[0].shape[0]
    n_fields = len(columns)
    header_overhead = 6 + 2 * n_fields  # packetType/flags/len/count + per-stream lengths
    bytes_per_record = 4 * n_fields  # float32 per field
    max_records = max(1, (MAX_PACKET_LOGICAL_BYTES - header_overhead) // bytes_per_record)

    out = bytearray()
    offset = 0
    while offset < n or (n == 0 and offset == 0):
        count = min(max_records, n - offset) if n > 0 else 0
        streams = [np.ascontiguousarray(col[offset:offset + count], dtype="<f4").tobytes() for col in columns]
        raw_len = header_overhead + sum(len(s) for s in streams)
        pad = (-raw_len) % 4
        pkt_len = raw_len + pad
        out += struct.pack("<BBHH", 1, 0, pkt_len - 1, n_fields)
        for s in streams:
            out += struct.pack("<H", len(s))
        for s in streams:
            out += s
        out += b"\x00" * pad
        offset += count
        if n == 0:
            break
    return bytes(out)


def _xml_escape(text: str) -> str:
    return (
        text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    )


# The four fields this project has always captured. Anything past these
# (e.g. "ring"/"time", see node.py's EXTRA_POINT_FIELDS) is optional and
# purely additive -- write_e57 doesn't otherwise care what they are.
DEFAULT_FIELD_NAMES = ("x", "y", "z", "intensity")

# PCD field name -> the real E57 prototype name a reader actually
# expects for the four standard ones (E57's own Data3D convention).
# Anything not in this map (ring, time, ...) is written verbatim as its
# own CompressedVector field, same "Float precision=single" encoding as
# everything else -- not one of E57's own standard names, but the
# format's prototype isn't a fixed enum; a real reader that doesn't
# recognize a field name just doesn't know what to do with it, same as
# any other schema-on-read format, rather than refusing the whole file
# (confirmed empirically against pye57/libE57Format, see HANDOFF.md).
_E57_PROTOTYPE_NAMES = {"x": "cartesianX", "y": "cartesianY", "z": "cartesianZ"}


def _bounds(col: np.ndarray) -> tuple[float, float]:
    """(0.0, 0.0) for an empty column or one that's entirely NaN (e.g.
    an EXTRA_POINT_FIELDS column on a run where the source topic didn't
    actually carry it, see node.py) -- an XML attribute literally
    reading minimum="nan" is asking for trouble from a stricter reader,
    and there's no real bound to report anyway in that case."""
    if col.size == 0 or not np.isfinite(col).any():
        return 0.0, 0.0
    return float(np.nanmin(col)), float(np.nanmax(col))


def write_e57(
    path: str,
    points: np.ndarray,
    field_names: tuple[str, ...] = DEFAULT_FIELD_NAMES,
    metadata: Optional[dict] = None,
) -> None:
    """Writes an Nx(len(field_names)) float array as a single-scan ASTM
    E57 file -- field_names must start with DEFAULT_FIELD_NAMES (x, y,
    z, intensity all matter to the surrounding metadata: cartesianBounds,
    intensityLimits, pose), with anything past that (ring, time, ...)
    riding along as additional prototype/CompressedVector fields, same
    "Float precision=single" encoding, no special handling needed since
    _build_data_packets already generalizes over an arbitrary field list.
    A column that's entirely NaN (see node.py's EXTRA_POINT_FIELDS
    fallback) still gets written -- a reader sees NaN values with a
    (0.0, 0.0) bound, self-explanatory as "not really populated" rather
    than silently dropped.

    metadata (all optional) may include: station_name, description,
    sensor_vendor, sensor_model, sensor_serial, acquisition_start_unix,
    acquisition_end_unix (float unix timestamps -- defaults to now/now
    if omitted), coordinate_metadata (free text, e.g. a note on the
    output frame/units).

    Pose is written as identity (no rotation, no translation) -- points
    is expected to already be in scan_aggregator's own output_frame (see
    _transform_and_accumulate), the same convention write_pcd's own
    output uses, so there is no separate scan-to-world transform to
    record here.
    """
    if field_names[: len(DEFAULT_FIELD_NAMES)] != DEFAULT_FIELD_NAMES:
        raise ValueError(f"field_names must start with {DEFAULT_FIELD_NAMES}, got {field_names}")
    metadata = metadata or {}
    n = points.shape[0]
    pts = np.ascontiguousarray(points, dtype=np.float32)
    columns = [pts[:, i] for i in range(len(field_names))]
    x, y, z, intensity = columns[0], columns[1], columns[2], columns[3]
    field_bounds = [_bounds(col) for col in columns]

    x_min, x_max = field_bounds[0]
    y_min, y_max = field_bounds[1]
    z_min, z_max = field_bounds[2]
    i_min, i_max = field_bounds[3]

    now = time.time()
    acq_start = float(metadata.get("acquisition_start_unix", now))
    acq_end = float(metadata.get("acquisition_end_unix", acq_start))
    station_name = _xml_escape(str(metadata.get("station_name", "scan")))
    description = _xml_escape(str(metadata.get("description", "TPL (Terrestrial Panning Lidar) scan_aggregator output")))
    sensor_vendor = _xml_escape(str(metadata.get("sensor_vendor", "Velodyne")))
    sensor_model = _xml_escape(str(metadata.get("sensor_model", "VLP-16")))
    sensor_serial = metadata.get("sensor_serial")
    coordinate_metadata = _xml_escape(str(metadata.get("coordinate_metadata", "")))
    file_guid = f"{{{uuid.uuid4()}}}"
    scan_guid = f"{{{uuid.uuid4()}}}"

    # Arbitrary extra {element_name: text} pairs, rendered as their own
    # sibling String elements after <description> -- e.g. node.py passes
    # real scan config / mount calibration / raw sensor status through
    # here as separate machine-readable fields rather than burying JSON
    # inside the human-readable description text. Not standard E57
    # element names, but neither is "ring"/"time" on the CompressedVector
    # side (see _E57_PROTOTYPE_NAMES) -- same reasoning: a real reader
    # that doesn't recognize an element name just ignores it rather than
    # refusing the file, confirmed against pye57/libE57Format.
    extra_fields_xml = "".join(
        f'\n      <{name} type="String"><![CDATA[{_xml_escape(str(value))}]]></{name}>'
        for name, value in (metadata.get("extra_string_fields") or {}).items()
    )

    # CompressedVector section: placed immediately after the 48-byte file
    # header, matching the reference implementation's own layout. Its own
    # dataPhysicalOffset is always 80 in practice (48-byte header + this
    # 32-byte section header, always well inside page 0 for this
    # project's layout -- no scan is ever small enough to matter), but
    # still run through _logical_to_physical_offset rather than left as
    # a bare literal, for the same reason xml_physical_offset below must
    # be: it's a field actually named "...Physical...", not a value this
    # module gets to assume stays a no-op just because it always has so
    # far (see that function's own docstring for how this was found).
    packets = _build_data_packets(columns)
    section_logical_length = 32 + len(packets)
    data_physical_offset = _logical_to_physical_offset(48 + 32)
    section_header = struct.pack(
        "<B7xQQQ", 1, section_logical_length, data_physical_offset, 0
    )
    section_bytes = section_header + packets
    points_file_offset = 48

    sensor_serial_xml = (
        f'\n      <sensorSerialNumber type="String"><![CDATA[{_xml_escape(str(sensor_serial))}]]></sensorSerialNumber>'
        if sensor_serial else ""
    )

    prototype_entries = "\n".join(
        f'          <{_E57_PROTOTYPE_NAMES.get(name, name)} type="Float" '
        f'precision="single" minimum="{lo!r}" maximum="{hi!r}"/>'
        for name, (lo, hi) in zip(field_names, field_bounds)
    )

    xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<e57Root type="Structure" xmlns="http://www.astm.org/COMMIT/E57/2010-e57-v1.0">
  <formatName type="String"><![CDATA[ASTM E57 3D Imaging Data File]]></formatName>
  <guid type="String"><![CDATA[{file_guid}]]></guid>
  <versionMajor type="Integer">1</versionMajor>
  <versionMinor type="Integer">0</versionMinor>
  <e57LibraryVersion type="String"><![CDATA[TPL e57_writer.py (hand-rolled, see HANDOFF.md)]]></e57LibraryVersion>
  <coordinateMetadata type="String"><![CDATA[{coordinate_metadata}]]></coordinateMetadata>
  <creationDateTime type="Structure">
    <dateTimeValue type="Float">{now!r}</dateTimeValue>
    <isAtomicClockReferenced type="Integer">0</isAtomicClockReferenced>
  </creationDateTime>
  <data3D type="Vector" allowHeterogeneousChildren="1">
    <vectorChild type="Structure">
      <guid type="String"><![CDATA[{scan_guid}]]></guid>
      <name type="String"><![CDATA[{station_name}]]></name>
      <description type="String"><![CDATA[{description}]]></description>{extra_fields_xml}
      <sensorVendor type="String"><![CDATA[{sensor_vendor}]]></sensorVendor>
      <sensorModel type="String"><![CDATA[{sensor_model}]]></sensorModel>{sensor_serial_xml}
      <indexBounds type="Structure">
        <rowMinimum type="Integer">0</rowMinimum>
        <rowMaximum type="Integer">{max(n - 1, 0)}</rowMaximum>
      </indexBounds>
      <intensityLimits type="Structure">
        <intensityMinimum type="Float">{i_min!r}</intensityMinimum>
        <intensityMaximum type="Float">{i_max!r}</intensityMaximum>
      </intensityLimits>
      <cartesianBounds type="Structure">
        <xMinimum type="Float">{x_min!r}</xMinimum>
        <xMaximum type="Float">{x_max!r}</xMaximum>
        <yMinimum type="Float">{y_min!r}</yMinimum>
        <yMaximum type="Float">{y_max!r}</yMaximum>
        <zMinimum type="Float">{z_min!r}</zMinimum>
        <zMaximum type="Float">{z_max!r}</zMaximum>
      </cartesianBounds>
      <pose type="Structure">
        <rotation type="Structure">
          <w type="Float">1</w>
          <x type="Float">0</x>
          <y type="Float">0</y>
          <z type="Float">0</z>
        </rotation>
        <translation type="Structure">
          <x type="Float">0</x>
          <y type="Float">0</y>
          <z type="Float">0</z>
        </translation>
      </pose>
      <acquisitionStart type="Structure">
        <dateTimeValue type="Float">{acq_start!r}</dateTimeValue>
        <isAtomicClockReferenced type="Integer">0</isAtomicClockReferenced>
      </acquisitionStart>
      <acquisitionEnd type="Structure">
        <dateTimeValue type="Float">{acq_end!r}</dateTimeValue>
        <isAtomicClockReferenced type="Integer">0</isAtomicClockReferenced>
      </acquisitionEnd>
      <points type="CompressedVector" fileOffset="{points_file_offset}" recordCount="{n}">
        <prototype type="Structure">
{prototype_entries}
        </prototype>
        <codecs type="Vector" allowHeterogeneousChildren="1">
        </codecs>
      </points>
    </vectorChild>
  </data3D>
  <images2D type="Vector" allowHeterogeneousChildren="1">
  </images2D>
</e57Root>
"""
    xml_bytes = xml.encode("utf-8")

    header_placeholder = b"\x00" * 48
    logical_content = header_placeholder + section_bytes + xml_bytes
    xml_physical_offset = _logical_to_physical_offset(48 + len(section_bytes))
    xml_logical_length = len(xml_bytes)

    padded = _pad_to_page_boundary(logical_content)
    n_pages = len(padded) // PAGE_PAYLOAD
    file_physical_length = n_pages * PAGE_SIZE

    real_header = struct.pack(
        "<8sIIQQQQ",
        b"ASTM-E57",
        1,
        0,
        file_physical_length,
        xml_physical_offset,
        xml_logical_length,
        PAGE_SIZE,
    )
    padded = real_header + padded[48:]

    physical = _physical_bytes_from_logical(padded)
    with open(path, "wb") as f:
        f.write(physical)
        f.flush()
        os.fsync(f.fileno())


def read_pcd_points(path: str) -> tuple[np.ndarray, tuple[str, ...]]:
    """Reads an Nx(field count) float32 array back out of a PCD file,
    plus the field names actually declared in its header -- the inverse
    of pcd_writer.write_pcd, whatever field_names it was called with
    (the original x/y/z/intensity-only files this project wrote before
    2026-09-10, and the x/y/z/intensity/ring/time ones since -- see
    node.py's POINT_FIELD_NAMES). Also accepts `DATA ascii` (pre-
    2026-08-28 scans, and anything hand-exported by another tool)
    alongside the `DATA binary` this project's own writer has produced
    since then, since real files of both kinds exist on disk from this
    project's own history and both are worth being able to convert.
    Assumes every field is SIZE 4/TYPE F/COUNT 1 (float32), same as
    every file write_pcd has ever produced -- doesn't attempt to handle
    an arbitrary third-party PCD with a genuinely different field
    layout (e.g. packed RGB, double-precision)."""
    with open(path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: EOF before DATA line -- not a valid PCD header")
            header_lines.append(line)
            if line.startswith(b"DATA"):
                break
        header = b"".join(header_lines).decode("ascii", errors="replace")

        fields = None
        sizes = None
        types = None
        counts = None
        width = None
        data_kind = None
        for raw_line in header.splitlines():
            parts = raw_line.split()
            if not parts:
                continue
            key = parts[0].upper()
            if key == "FIELDS":
                fields = tuple(parts[1:])
            elif key == "SIZE":
                sizes = parts[1:]
            elif key == "TYPE":
                types = parts[1:]
            elif key == "COUNT":
                counts = parts[1:]
            elif key == "WIDTH":
                width = int(parts[1])
            elif key == "DATA":
                data_kind = parts[1].lower()

        if not fields:
            raise ValueError(f"{path}: missing FIELDS in header")
        if width is None:
            raise ValueError(f"{path}: missing WIDTH in header")
        if (
            sizes != ["4"] * len(fields)
            or types != ["F"] * len(fields)
            or counts != ["1"] * len(fields)
        ):
            raise ValueError(
                f"{path}: unsupported field layout (expected every field float32, "
                f"count 1) -- SIZE={sizes} TYPE={types} COUNT={counts}"
            )
        n_fields = len(fields)

        if data_kind == "binary":
            raw = f.read(width * 4 * n_fields)
            arr = np.frombuffer(raw, dtype="<f4").reshape(width, n_fields)
            return np.ascontiguousarray(arr, dtype=np.float32), fields
        elif data_kind == "ascii":
            arr = np.loadtxt(f, dtype=np.float32, max_rows=width)
            if arr.ndim == 1:
                arr = arr.reshape(-1, n_fields)
            return np.ascontiguousarray(arr, dtype=np.float32), fields
        else:
            raise ValueError(f"{path}: unsupported DATA kind {data_kind!r}")
