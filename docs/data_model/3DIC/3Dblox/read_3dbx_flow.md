# `read_3dbx` Command Flow

This document traces the complete execution path of the `read_3dbx` Tcl command,
from user input through YAML parsing, OpenDB population, observer notification,
and automatic design rule checking.

## Call Chain Overview

```
read_3dbx "design.3dbx"          (Tcl: OpenRoad.tcl:203)
  -> ord::read_3dbx_cmd           (SWIG: OpenRoad.i:377)
    -> OpenRoad::read3Dbx()       (C++: OpenRoad.cc:520)
      -> ThreeDBlox::readDbx()   (3dblox.cpp:176)
        1. DbxParser::parseFile()       -- YAML parse into DbxData
        2. readHeaderIncludes()         -- recursively load .3dbv/.3dbx
        3. createDesignTopChiplet()     -- create top-level dbChip(HIER)
        4. createChipInst()             -- instantiate chiplets with 3D placement
        5. buildChipNetsFromVerilog()   -- create dbChipNet from Verilog netlist
        6. createConnection()           -- create dbChipConn between regions
        7. calculateSize()              -- compute top chip bounding box
        8. create path assertions       -- dbChipPath entries
      -> triggerPostRead3Dbx()          -- notify observers (dbSta, GUI)
      -> check3DBlox()                  -- automatic DRC (8 checks)
```

## Step-by-Step Detail

### 1. Tcl Validation (`OpenRoad.tcl:203-214`)

```tcl
proc read_3dbx { args } {
  sta::parse_key_args "read_3dbx" args keys {} flags {}
  sta::check_argc_eq1 "read_3dbx" $args
  set filename [file nativename [lindex $args 0]]
  # check file exists and is readable, then call C++
  ord::read_3dbx_cmd $filename
}
```

Validates exactly one argument, checks file existence/readability.

### 2. SWIG Bridge (`OpenRoad.i:377-381`)

```cpp
void read_3dbx_cmd(const char *filename) {
  OpenRoad *ord = getOpenRoad();
  ord->read3Dbx(filename);
}
```

### 3. C++ Entry Point (`OpenRoad.cc:520-526`)

```cpp
void OpenRoad::read3Dbx(const std::string& filename) {
  odb::ThreeDBlox parser(logger_, db_, sta_);
  parser.readDbx(filename);
  db_->triggerPostRead3Dbx(db_->getChip());
  check3DBlox();
}
```

### 4. YAML Parsing (`dbxParser.cpp:24-43`)

`DbxParser::parseFile()` does:
1. **`parseDefines()`** -- text-level `#!define` macro expansion (not YAML)
2. **`YAML::Load()`** -- parse the processed string
3. Extract sections into `DbxData` struct:
   - `Header` -- version, unit, precision, include list
   - `Design` -- design name, verilog file
   - `ChipletInst` -- chiplet instance references
   - `Stack` -- 3D placement (x, y, z, orient)
   - `Connections` -- top/bottom region paths + thickness
   - `Paths` -- path assertions (optional `NOT` prefix)

Data structures are defined in `objects.h`.

### 5. Recursive Includes (`3dblox.cpp:351-372`)

`readHeaderIncludes()` processes `Header.include` list:
- `.3dbv` files -> `readDbv()` -> `DbvParser` -> `createChiplet()`
- `.3dbx` files -> `readDbx()` (recursive)
- De-duplicates via `std::unordered_set<std::string> read_files_`

### 6. Chiplet Creation via `.3dbv` (`3dblox.cpp:402-518`)

`createChiplet()` for each chiplet definition:
- Reads tech LEF (`lefin::createTechAndLib()`)
- Reads library LEF (`lefin::createLib()`)
- Reads Liberty (`sta_->readLiberty()`)
- Creates `dbChip` with type (DIE, RDL, IP, SUBSTRATE, HIER)
- Reads DEF (`defin::readChip()`)
- Sets dimensions, thickness, shrink, TSV, seal ring, scribe line
- Creates `dbChipRegion` objects with bounding boxes
- Parses `.bmap` to create bump cells

### 7. Top Chip + Instances (`3dblox.cpp:656-701`)

- **`createDesignTopChiplet()`** -- creates `dbChip(HIER)`, stores verilog file, calls `db_->setTopChip()`
- **`createChipInst()`** -- looks up master chip via `db_->findChip()`, creates `dbChipInst` with 3D orient and location (converted to DBU)

### 8. Net and Connection Building (`3dblox.cpp:88-174, 771-786`)

- **`buildChipNetsFromVerilog()`** -- uses OpenSTA's `VerilogReader` to parse netlist, creates `dbChipNet`, links bumps to nets by port name matching
- **`createConnection()`** -- resolves hierarchical paths (e.g., `soc_inst.regions.front_reg`) to `dbChipRegionInst`, creates `dbChipConn` with thickness

### 9. Size Calculation (`3dblox.cpp:339-349`)

`calculateSize()` merges all chip instance cuboids to compute the top chip's bounding box.

### 10. Observer Notification (`dbDatabase.cpp:1040-1045`)

`triggerPostRead3Dbx()` iterates `dbDatabaseObserver` list:
- **dbSta** (`dbSta.cc:384`) -- stub (TODO: timing on chiplets)
- **GUI** (`mainWindow.cpp:1647`) -- emits `chipLoaded(chip)` signal

### 11. Automatic DRC (`checker.cpp`)

`check3DBlox()` builds an `UnfoldedModel` (flattened 3D coordinates) and runs:
1. `checkLogicalConnectivity` -- bump net mismatches
2. `checkFloatingChips` -- unconnected chips
3. `checkOverlappingChips` -- 3D bounding box overlaps
4. `checkInternalExtUsage` -- unused INTERNAL_EXT regions
5. `checkConnectionRegions` -- invalid endpoints
6. `checkBumpPhysicalAlignment` -- bumps outside parent region
7. `checkNetConnectivity` -- (stub)
8. `checkAlignmentMarkers` -- marker cell pairs within tolerance

## Key Source Files

| File | Role |
|------|------|
| `src/OpenRoad.tcl:201-214` | Tcl command definition |
| `src/OpenRoad.i:377-381` | SWIG bridge |
| `include/ord/OpenRoad.hh:234` | Method declaration |
| `src/OpenRoad.cc:520-526` | C++ entry point |
| `src/odb/include/odb/3dblox.h:37-72` | ThreeDBlox class API |
| `src/odb/src/3dblox/3dblox.cpp:176-202` | readDbx core logic |
| `src/odb/src/3dblox/dbxParser.cpp` | 3DBX YAML parsing |
| `src/odb/src/3dblox/dbvParser.cpp` | 3DBV YAML parsing |
| `src/odb/src/3dblox/baseParser.cpp` | Shared parser base |
| `src/odb/src/3dblox/objects.h` | Data structures |
| `src/odb/src/3dblox/bmapParser.cpp` | Bump map parser |
| `src/odb/src/3dblox/checker.cpp` | Design rule checker |
| `src/odb/include/odb/dbDatabaseObserver.h` | Observer interface |
| `src/odb/src/db/dbDatabase.cpp:1040-1045` | triggerPostRead3Dbx |
| `src/dbSta/src/dbSta.cc:384-387` | dbSta observer (stub) |
| `src/gui/src/mainWindow.cpp:1647-1650` | GUI observer |

## Related Commands

| Command | Purpose |
|---------|---------|
| `read_3dbv` | Read chiplet definitions (.3dbv) |
| `read_3dbx` | Read design assembly (.3dbx) |
| `write_3dbv` | Write chiplet definitions |
| `write_3dbx` | Write design assembly |
| `read_3dblox_bmap` | Read bump map (.bmap) |
| `check_3dblox` | Run design rule checker |
