/////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2019, The Regents of the University of California
// All rights reserved.
//
// BSD 3-Clause License
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#include "ord/OpenRoad.hh"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

#include "ord/Version.hh"
#ifdef ENABLE_PYTHON3
#define PY_SSIZE_T_CLEAN
#include "Python.h"
#endif

#include "ant/MakeAntennaChecker.hh"
#include "cts/MakeTritoncts.h"
#include "db_sta/MakeDbSta.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbReadVerilog.hh"
#include "db_sta/dbSta.hh"
#include "dft/MakeDft.hh"
#include "dpl/MakeOpendp.h"
#include "dpo/MakeOptdp.h"
#include "dst/MakeDistributed.h"
#include "fin/MakeFinale.h"
#include "gpl/MakeReplace.h"
#include "grt/MakeGlobalRouter.h"
#include "gui/MakeGui.h"
#include "ifp//MakeInitFloorplan.hh"
#include "mpl/MakeMacroPlacer.h"
#include "mpl2/MakeMacroPlacer.h"
#include "odb/cdl.h"
#include "odb/db.h"
#include "odb/defin.h"
#include "odb/defout.h"
#include "odb/lefin.h"
#include "odb/lefout.h"
#include "ord/InitOpenRoad.hh"
#include "pad/MakeICeWall.h"
#include "par/MakePartitionMgr.h"
#include "pdn/MakePdnGen.hh"
#include "ppl/MakeIoplacer.h"
#include "psm/MakePDNSim.hh"
#include "rcx/MakeOpenRCX.h"
#include "rmp/MakeRestructure.h"
#include "rsz/MakeResizer.hh"
#include "sta/StaMain.hh"
#include "sta/VerilogWriter.hh"
#include "stt/MakeSteinerTreeBuilder.h"
#include "tap/MakeTapcell.h"
#include "triton_route/MakeTritonRoute.h"
#include "utl/Logger.h"
#include "utl/MakeLogger.h"
#include "utl/ScopedTemporaryFile.h"
// MY CODE
#include "sta/ConcreteLibrary.hh"
#include "sta/PortDirection.hh"

namespace sta {
extern const char* openroad_swig_tcl_inits[];
extern const char* upf_tcl_inits[];
}  // namespace sta

// Swig uses C linkage for init functions.
extern "C" {
extern int Openroad_swig_Init(Tcl_Interp* interp);
extern int Odbtcl_Init(Tcl_Interp* interp);
extern int Upf_Init(Tcl_Interp* interp);
}

namespace ord {

using odb::dbBlock;
using odb::dbChip;
using odb::dbDatabase;
using odb::dbLib;
using odb::dbTech;
using odb::Rect;

using sta::evalTclInit;

using utl::ORD;

OpenRoad* OpenRoad::app_ = nullptr;

OpenRoad::OpenRoad()
{
  db_ = dbDatabase::create();
}

OpenRoad::~OpenRoad()
{
  deleteDbVerilogNetwork(verilog_network_);
  // Temporarily removed until a crash can be resolved
  // deleteDbSta(sta_);
  // sta::deleteAllMemory();
  deleteIoplacer(ioPlacer_);
  deleteResizer(resizer_);
  deleteOpendp(opendp_);
  deleteOptdp(optdp_);
  deleteGlobalRouter(global_router_);
  deleteRestructure(restructure_);
  deleteTritonCts(tritonCts_);
  deleteTapcell(tapcell_);
  deleteMacroPlacer(macro_placer_);
  deleteMacroPlacer2(macro_placer2_);
  deleteOpenRCX(extractor_);
  deleteTritonRoute(detailed_router_);
  deleteReplace(replace_);
  deleteFinale(finale_);
  deleteAntennaChecker(antenna_checker_);
  odb::dbDatabase::destroy(db_);
  deletePartitionMgr(partitionMgr_);
  deletePdnGen(pdngen_);
  deleteICeWall(icewall_);
  deleteDistributed(distributer_);
  deleteSteinerTreeBuilder(stt_builder_);
  dft::deleteDft(dft_);
  delete logger_;
}

sta::dbNetwork* OpenRoad::getDbNetwork()
{
  return sta_->getDbNetwork();
}

/* static */
OpenRoad* OpenRoad::openRoad()
{
  return app_;
}

/* static */
void OpenRoad::setOpenRoad(OpenRoad* app, bool reinit_ok)
{
  if (!reinit_ok && app_) {
    std::cerr << "Attempt to reinitialize the application." << std::endl;
    exit(1);
  }
  app_ = app;
}

////////////////////////////////////////////////////////////////

void initOpenRoad(Tcl_Interp* interp,
                  const char* log_filename,
                  const char* metrics_filename)
{
  OpenRoad::openRoad()->init(interp, log_filename, metrics_filename);
}

void OpenRoad::init(Tcl_Interp* tcl_interp,
                    const char* log_filename,
                    const char* metrics_filename)
{
  tcl_interp_ = tcl_interp;

  // Make components.
  logger_ = makeLogger(log_filename, metrics_filename);
  db_->setLogger(logger_);
  sta_ = makeDbSta();
  verilog_network_ = makeDbVerilogNetwork();
  ioPlacer_ = makeIoplacer();
  resizer_ = makeResizer();
  opendp_ = makeOpendp();
  optdp_ = makeOptdp();
  finale_ = makeFinale();
  global_router_ = makeGlobalRouter();
  restructure_ = makeRestructure();
  tritonCts_ = makeTritonCts();
  tapcell_ = makeTapcell();
  macro_placer_ = makeMacroPlacer();
  macro_placer2_ = makeMacroPlacer2();
  extractor_ = makeOpenRCX();
  detailed_router_ = makeTritonRoute();
  replace_ = makeReplace();
  pdnsim_ = makePDNSim();
  antenna_checker_ = makeAntennaChecker();
  partitionMgr_ = makePartitionMgr();
  pdngen_ = makePdnGen();
  icewall_ = makeICeWall();
  distributer_ = makeDistributed();
  stt_builder_ = makeSteinerTreeBuilder();
  dft_ = dft::makeDft();

  // Init components.
  Openroad_swig_Init(tcl_interp);
  // Import TCL scripts.
  evalTclInit(tcl_interp, sta::openroad_swig_tcl_inits);

  initLogger(logger_, tcl_interp);
  initGui(this);  // first so we can register our sink with the logger
  Odbtcl_Init(tcl_interp);
  Upf_Init(tcl_interp);
  evalTclInit(tcl_interp, sta::upf_tcl_inits);
  initInitFloorplan(this);
  initDbSta(this);
  initResizer(this);
  initDbVerilogNetwork(this);
  initIoplacer(this);
  initReplace(this);
  initOpendp(this);
  initOptdp(this);
  initFinale(this);
  initGlobalRouter(this);
  initTritonCts(this);
  initTapcell(this);
  initMacroPlacer(this);
  initMacroPlacer2(this);
  initOpenRCX(this);
  initICeWall(this);
  initRestructure(this);
  initTritonRoute(this);
  initPDNSim(this);
  initAntennaChecker(this);
  initPartitionMgr(this);
  initPdnGen(this);
  initDistributed(this);
  initSteinerTreeBuilder(this);
  dft::initDft(this);

  // Import exported commands to global namespace.
  Tcl_Eval(tcl_interp, "sta::define_sta_cmds");
  Tcl_Eval(tcl_interp, "namespace import sta::*");

  // Initialize tcl history
  if (Tcl_Eval(tcl_interp, "history") == TCL_ERROR) {
    // There appears to be a typo in the history.tcl file in some
    // distributions, which is generating this error.
    // remove error from tcl result.
    Tcl_ResetResult(tcl_interp);
  }
}

////////////////////////////////////////////////////////////////

void OpenRoad::readLef(const char* filename,
                       const char* lib_name,
                       const char* tech_name,
                       bool make_tech,
                       bool make_library)
{
  odb::lefin lef_reader(db_, logger_, false);
  dbLib* lib = nullptr;
  dbTech* tech = nullptr;
  if (make_tech && make_library) {
    lib = lef_reader.createTechAndLib(tech_name, lib_name, filename);
    tech = db_->findTech(tech_name);
  } else if (make_tech) {
    tech = lef_reader.createTech(tech_name, filename);
  } else if (make_library) {
    if (tech_name[0] != '\0') {
      tech = db_->findTech(tech_name);
    } else {
      tech = db_->getTech();
    }
    if (!tech) {
      logger_->error(ORD, 51, "Technology {} not found", tech_name);
    }
    lib = lef_reader.createLib(tech, lib_name, filename);
  }

  // both are null on parser failure
  if (lib != nullptr || tech != nullptr) {
    for (OpenRoadObserver* observer : observers_) {
      observer->postReadLef(tech, lib);
    }
  }
}

void OpenRoad::readDef(const char* filename,
                       dbTech* tech,
                       bool continue_on_errors,
                       bool floorplan_init,
                       bool incremental,
                       bool child)
{
  if (!floorplan_init && !incremental && !child && db_->getChip()
      && db_->getChip()->getBlock()) {
    logger_->info(ORD, 48, "Loading an additional DEF.");
  }

  odb::defin::MODE mode = odb::defin::DEFAULT;
  if (floorplan_init) {
    mode = odb::defin::FLOORPLAN;
  } else if (incremental) {
    mode = odb::defin::INCREMENTAL;
  }
  odb::defin def_reader(db_, logger_, mode);
  std::vector<odb::dbLib*> search_libs;
  for (odb::dbLib* lib : db_->getLibs()) {
    search_libs.push_back(lib);
  }
  if (continue_on_errors) {
    def_reader.continueOnErrors();
  }
  dbBlock* block = nullptr;
  if (child) {
    auto parent = db_->getChip()->getBlock();
    block = def_reader.createBlock(parent, search_libs, filename, tech);
  } else {
    dbChip* chip = def_reader.createChip(search_libs, filename, tech);
    if (chip) {
      block = chip->getBlock();
    }
  }
  if (block) {
    for (OpenRoadObserver* observer : observers_) {
      observer->postReadDef(block);
    }
  }
}

static odb::defout::Version stringToDefVersion(const string& version)
{
  if (version == "5.8") {
    return odb::defout::Version::DEF_5_8;
  }
  if (version == "5.7") {
    return odb::defout::Version::DEF_5_7;
  }
  if (version == "5.6") {
    return odb::defout::Version::DEF_5_6;
  }
  if (version == "5.5") {
    return odb::defout::Version::DEF_5_5;
  }
  if (version == "5.4") {
    return odb::defout::Version::DEF_5_4;
  }
  if (version == "5.3") {
    return odb::defout::Version::DEF_5_3;
  }
  return odb::defout::Version::DEF_5_8;
}

void OpenRoad::writeDef(const char* filename, const string& version)
{
  odb::dbChip* chip = db_->getChip();
  if (chip) {
    odb::dbBlock* block = chip->getBlock();
    if (block) {
      sta::dbSta* sta = getSta();
      // def names are flat hierachical
      bool hierarchy_set = sta->getDbNetwork()->hasHierarchy();
      if (hierarchy_set) {
        sta->getDbNetwork()->disableHierarchy();
      }
      odb::defout def_writer(logger_);
      def_writer.setVersion(stringToDefVersion(version));
      def_writer.writeBlock(block, filename);
      if (hierarchy_set) {
        sta->getDbNetwork()->setHierarchy();
      }
    }
  }
}

void OpenRoad::writeAbstractLef(const char* filename,
                                const int bloat_factor,
                                const bool bloat_occupied_layers)
{
  odb::dbBlock* block = nullptr;
  odb::dbChip* chip = db_->getChip();
  if (chip) {
    block = chip->getBlock();
  }
  if (!block) {
    logger_->error(ORD, 53, "No block is loaded.");
  }
  utl::StreamHandler stream_handler(filename);
  odb::lefout writer(logger_, stream_handler.getStream());
  writer.setBloatFactor(bloat_factor);
  writer.setBloatOccupiedLayers(bloat_occupied_layers);
  writer.writeAbstractLef(block);
}

void OpenRoad::writeLef(const char* filename)
{
  sta::dbSta* sta = getSta();
  bool hierarchy_set = sta->getDbNetwork()->hasHierarchy();
  if (hierarchy_set) {
    sta->getDbNetwork()->disableHierarchy();
  }
  auto libs = db_->getLibs();
  int num_libs = libs.size();
  if (num_libs > 0) {
    if (num_libs > 1) {
      logger_->info(
          ORD, 34, "More than one lib exists, multiple files will be written.");
    }
    int cnt = 0;
    for (auto lib : libs) {
      std::string name(filename);
      if (cnt > 0) {
        auto pos = name.rfind('.');
        if (pos != string::npos) {
          name.insert(pos, "_" + std::to_string(cnt));
        } else {
          name += "_" + std::to_string(cnt);
        }
        utl::StreamHandler stream_handler(name.c_str());
        odb::lefout lef_writer(logger_, stream_handler.getStream());
        lef_writer.writeLib(lib);
      } else {
        utl::StreamHandler stream_handler(filename);
        odb::lefout lef_writer(logger_, stream_handler.getStream());
        lef_writer.writeTechAndLib(lib);
      }
      ++cnt;
    }
  } else if (db_->getTech()) {
    utl::StreamHandler stream_handler(filename);
    odb::lefout lef_writer(logger_, stream_handler.getStream());
    lef_writer.writeTech(db_->getTech());
  }
  if (hierarchy_set) {
    sta->getDbNetwork()->setHierarchy();
  }
}

void OpenRoad::writeCdl(const char* outFilename,
                        const std::vector<const char*>& mastersFilenames,
                        bool includeFillers)
{
  odb::dbChip* chip = db_->getChip();
  if (chip) {
    odb::dbBlock* block = chip->getBlock();
    if (block) {
      odb::cdl::writeCdl(
          getLogger(), block, outFilename, mastersFilenames, includeFillers);
    }
  }
}

void OpenRoad::readDb(const char* filename, bool hierarchy)
{
  std::ifstream stream;
  stream.open(filename, std::ios::binary);
  try {
    readDb(stream);
  } catch (const std::ios_base::failure& f) {
    logger_->error(ORD, 54, "odb file {} is invalid: {}", filename, f.what());
  }
  // treat this as a hierarchical network.
  if (hierarchy) {
    sta::dbSta* sta = getSta();
    // After streaming in the last thing we do is build the hashes
    // we cannot rely on orders to do this during stream in
    sta->getDbNetwork()->setHierarchy();
  }
}

void OpenRoad::readDb(std::istream& stream)
{
  if (db_->getChip() && db_->getChip()->getBlock()) {
    logger_->error(
        ORD, 47, "You can't load a new db file as the db is already populated");
  }

  stream.exceptions(std::ifstream::failbit | std::ifstream::badbit
                    | std::ios::eofbit);

  db_->read(stream);

  // this fixes up the database post read
  for (OpenRoadObserver* observer : observers_) {
    observer->postReadDb(db_);
  }
}

void OpenRoad::writeDb(std::ostream& stream)
{
  stream.exceptions(std::ofstream::failbit | std::ofstream::badbit);
  db_->write(stream);
}

void OpenRoad::writeDb(const char* filename)
{
  utl::StreamHandler stream_handler(filename, true);

  db_->write(stream_handler.getStream());
}

void OpenRoad::diffDbs(const char* filename1,
                       const char* filename2,
                       const char* diffs)
{
  std::ifstream stream1;
  stream1.exceptions(std::ifstream::failbit | std::ifstream::badbit
                     | std::ios::eofbit);
  stream1.open(filename1, std::ios::binary);

  std::ifstream stream2;
  stream2.exceptions(std::ifstream::failbit | std::ifstream::badbit
                     | std::ios::eofbit);
  stream2.open(filename2, std::ios::binary);

  FILE* out = fopen(diffs, "w");
  if (out == nullptr) {
    logger_->error(ORD, 105, "Can't open {}", diffs);
  }

  auto db1 = odb::dbDatabase::create();
  auto db2 = odb::dbDatabase::create();

  db1->read(stream1);
  db2->read(stream2);

  odb::dbDatabase::diff(db1, db2, out, 2);

  fclose(out);
}

void OpenRoad::readVerilog(const char* filename)
{
  verilog_network_->deleteTopInstance();
  dbReadVerilog(filename, verilog_network_);
}

void OpenRoad::linkDesign(const char* design_name, bool hierarchy)

{
  dbLinkDesign(design_name, verilog_network_, db_, logger_, hierarchy);
  if (hierarchy) {
    sta::dbSta* sta = getSta();
    sta->getDbNetwork()->setHierarchy();
  }
  for (OpenRoadObserver* observer : observers_) {
    observer->postReadDb(db_);
  }
}

void OpenRoad::designCreated()
{
  for (OpenRoadObserver* observer : observers_) {
    observer->postReadDb(db_);
  }
}

bool OpenRoad::unitsInitialized()
{
  // Units are set by the first liberty library read.
  return getDbNetwork()->defaultLibertyLibrary() != nullptr;
}

odb::Rect OpenRoad::getCore()
{
  return db_->getChip()->getBlock()->getCoreArea();
}

void OpenRoad::addObserver(OpenRoadObserver* observer)
{
  observer->set_unregister_observer(
      [this, observer] { removeObserver(observer); });
  observers_.insert(observer);
}

void OpenRoad::removeObserver(OpenRoadObserver* observer)
{
  observer->set_unregister_observer(nullptr);
  observers_.erase(observer);
}

void OpenRoad::setThreadCount(int threads, bool printInfo)
{
  int max_threads = std::thread::hardware_concurrency();
  if (max_threads == 0) {
    logger_->warn(ORD,
                  31,
                  "Unable to determine maximum number of threads.\n"
                  "One thread will be used.");
    max_threads = 1;
  }
  if (threads <= 0) {  // max requested
    threads = max_threads;
  } else if (threads > max_threads) {
    threads = max_threads;
  }
  threads_ = threads;

  if (printInfo) {
    logger_->info(ORD, 30, "Using {} thread(s).", threads_);
  }

  // place limits on tools with threads
  sta_->setThreadCount(threads_);
}

void OpenRoad::setThreadCount(const char* threads, bool printInfo)
{
  int max_threads = threads_;  // default, make no changes

  if (strcmp(threads, "max") == 0) {
    max_threads = -1;  // -1 is max cores
  } else {
    try {
      max_threads = std::stoi(threads);
    } catch (const std::invalid_argument&) {
      logger_->warn(
          ORD, 32, "Invalid thread number specification: {}.", threads);
    }
  }

  setThreadCount(max_threads, printInfo);
}

int OpenRoad::getThreadCount()
{
  return threads_;
}

std::string OpenRoad::getExePath() const
{
  // use tcl since it already has a cross platform implementation of this
  if (Tcl_Eval(tcl_interp_, "file normalize [info nameofexecutable]")
      == TCL_OK) {
    std::string path = Tcl_GetStringResult(tcl_interp_);
    Tcl_ResetResult(tcl_interp_);
    return path;
  }
  return "";
}

std::string OpenRoad::getDocsPath() const
{
  const std::string exe = getExePath();

  if (exe.empty()) {
    return "";
  }

  std::filesystem::path path(exe);

  // remove binary name
  path = path.parent_path();

  if (path.stem() == "src") {
    // remove build
    return path.parent_path().parent_path() / "docs";
  }

  // remove bin
  return path.parent_path() / "share" / "openroad" / "man";
}

const char* OpenRoad::getVersion()
{
  return OPENROAD_VERSION;
}

const char* OpenRoad::getGitDescribe()
{
  return OPENROAD_GIT_DESCRIBE;
}

bool OpenRoad::getGPUCompileOption()
{
  return GPU;
}

bool OpenRoad::getPythonCompileOption()
{
  return BUILD_PYTHON;
}

bool OpenRoad::getGUICompileOption()
{
  return BUILD_GUI;
}

bool OpenRoad::getChartsCompileOption()
{
  return ENABLE_CHARTS;
}

// FOR DEBUG
void OpenRoad::dumpDb()
{
  /* DB LEVEL*/
  debugPrint(logger_,
             utl::ODB,
             "dumpDb",
             1,
             "========================DUMP ODB========================");
  dbDatabase* odb = getDb();
  debugPrint(
      logger_, utl::ODB, "dumpDb", 1, "============ODB Statistic============");
  debugPrint(
      logger_, utl::ODB, "dumpDb", 1, "Masters: {}", odb->getNumberOfMasters());
  /* LIB LEVEL */
  /// dbLib is cell lef
  debugPrint(logger_, utl::ODB, "dumpDb", 1, "============dbLib============");
  for (dbLib* lib : odb->getLibs()) {
    debugPrint(logger_,
               utl::ODB,
               "dumpDb",
               1,
               "lib: {}, master: {}, lefUnits: {}",
               lib->getName(),
               lib->getMasters().size(),
               lib->getLefUnits());
  }
  /* TECH LEVEL */
  /// dbTech is tech lef
  debugPrint(logger_, utl::ODB, "dumpDb", 1, "============dbTech============");
  debugPrint(logger_,
             utl::ODB,
             "dumpDb",
             1,
             "odb can contain multiple techs of each dbBlock");
  debugPrint(logger_,
             utl::ODB,
             "dumpDb",
             1,
             "dbTech was recommended to get by dbBlock or dbLib");
  dbTech* tech = odb->getTech();
  debugPrint(logger_,
             utl::ODB,
             "dumpDb",
             1,
             "tech: {}, version: {}, layer: {}, routingLayer: {}, via: {}, "
             "unitsPerMicron: {}, lefUnits: {}",
             tech->getName(),
             tech->getLefVersionStr(),
             tech->getLayerCount(),
             tech->getRoutingLayerCount(),
             tech->getViaCount(),
             tech->getDbUnitsPerMicron(),
             tech->getLefUnits());
  /* TECH -> LAYER */
  for (auto layer : tech->getLayers()) {
    debugPrint(logger_,
               utl::ODB,
               "dumpDb",
               1,
               "layer: {}, area: {}, width: {}, pitch: {}, offset: {}, "
               "spacing: {}, direction: {}",
               layer->getName(),
               layer->getArea(),
               layer->getWidth(),
               layer->getPitch(),
               layer->getOffset(),
               layer->getSpacing(),
               layer->getDirection().getString());
  }
  /* TECH -> VIA */
  for (auto via : tech->getVias()) {
    debugPrint(logger_,
               utl::ODB,
               "dumpDb",
               1,
               "\nvia: {}, resistance: {}, topLayer: {}, bottomLayer: {}",
               via->getName(),
               via->getResistance(),
               via->getTopLayer()->getName(),
               via->getBottomLayer()->getName());
  }
  /* CHIP LEVEL */
  debugPrint(logger_, utl::ODB, "dumpDb", 1, "============dbChip============");
  dbChip* chip = odb->getChip();
  debugPrint(logger_, utl::ODB, "dumpDb", 1, "just get dbBlock by dbChip");
  /* BLOCK LEVEL */
  debugPrint(logger_, utl::ODB, "dumpDb", 1, "============dbBlock============");
  dbBlock* block = chip->getBlock();
  debugPrint(logger_,
             utl::ODB,
             "dumpDb",
             1,
             "dieArea: {}, coreArea: {}, module: {}, modInst: {}, modBTerms: "
             "{}, modNets: {}, instance: {}, bterms: {}, iterms: {},"
             " nets: {}, cornerCnt: {}, blockages: {}",
             block->getDieArea(),
             block->getCoreArea(),
             block->getModules().size(),
             block->getModInsts().size(),
             block->getModBTerms().size(),
             block->getModNets().size(),
             block->getInsts().size(),
             block->getBTerms().size(),
             block->getITerms().size(),
             block->getNets().size(),
             block->getCornerCount(),
             block->getBlockages().size());

  /* MODULE LEVEL */
  dumpDbModule(block->getTopModule());

  debugPrint(logger_,
             utl::ODB,
             "dumpDb",
             1,
             "========================DUMP ODB========================");
}

void OpenRoad::dumpDbModule(odb::dbModule* mod, int hier)
{
  if (mod == nullptr) {
    debugPrint(logger_, utl::ORD, "dumpDb", 1, "module is nullptr");
    return;
  }

  debugPrint(logger_,
             utl::ODB,
             "dumpDb",
             1,
             "============dbModule of hier {}============",
             hier);

  debugPrint(logger_,
             utl::ODB,
             "dumpDb",
             1,
             "dbModule {}, owner block: {}",
             mod->getHierarchicalName(),
             mod->getOwner()->getName());

  debugPrint(logger_,
             utl::ODB,
             "dumpDb",
             1,
             "============module connections============");
  // modNet and related terms
  debugPrint(logger_, utl::ODB, "dumpDb", 1, "mod->getModNets()");
  for (auto modNet : mod->getModNets()) {
    debugPrint(
        logger_, utl::ODB, "dumpDb", 1, "dbModNet {}", modNet->getName());

    for (auto modBTerm : modNet->getModBTerms()) {
      debugPrint(logger_,
                 utl::ODB,
                 "dumpDb",
                 1,
                 "\t dbModBTerm {}, dir {}",
                 modBTerm->getName(),
                 modBTerm->getIoType().getString());
    }

    for (auto modITerm : modNet->getModITerms()) {
      debugPrint(logger_,
                 utl::ODB,
                 "dumpDb",
                 1,
                 "\t dbModITerm {}, of dbModInst {}",
                 modITerm->getName(),
                 modITerm->getParent()->getHierarchicalName());
    }

    for (auto bTerm : modNet->getBTerms()) {
      debugPrint(
          logger_, utl::ODB, "dumpDb", 1, "\t dbBTerm {}", bTerm->getName());
    }

    for (auto iTerm : modNet->getITerms()) {
      debugPrint(logger_,
                 utl::ODB,
                 "dumpDb",
                 1,
                 "\t dbITerm {}, dir {}",
                 iTerm->getName(),
                 iTerm->getIoType().getString());
    }
  }

  debugPrint(logger_, utl::ODB, "dumpDb", 1, "mod->getModBTerms()");
  for (auto modBTerm : mod->getModBTerms()) {
    debugPrint(logger_,
               utl::ODB,
               "dumpDb",
               1,
               "dbModBTerm {}, type: {}",
               modBTerm->getName(),
               modBTerm->getIoType().getString());
  }

  debugPrint(logger_, utl::ODB, "dumpDb", 1, "module local hierarchy");
  debugPrint(logger_,
             utl::ODB,
             "dumpDb",
             1,
             "dbInstCnt in local hier: {}, dbModInstCnt in local hier: {}",
             mod->getDbInstCount(),
             mod->getModInstCount());

  debugPrint(logger_, utl::ODB, "dumpDb", 1, "mod->getInsts()");
  for (auto inst : mod->getInsts()) {
    debugPrint(logger_,
               utl::ODB,
               "dumpDb",
               1,
               "dbInst in local hier {}",
               inst->getName());
  }

  debugPrint(logger_,
             utl::ODB,
             "dumpDb",
             1,
             "mod->getLeafInsts() from entire hierarchy");
  for (auto leaf : mod->getLeafInsts()) {
    debugPrint(logger_,
               utl::ODB,
               "dumpDb",
               1,
               "leaf: {}, master: {}, area: {}, width: {}, weight: {}, level: "
               "{}, parent: "
               "{}, belonged mod: {}, isHierarchical: {}, "
               "placementStatus: {}",
               leaf->getName(),
               leaf->getMaster()->getName(),
               leaf->getMaster()->getArea(),
               leaf->getMaster()->getWidth(),
               leaf->getWeight(),
               leaf->getLevel(),
               leaf->getParent() ? leaf->getParent()->getName() : "empty",
               leaf->getModule()->getHierarchicalName(),
               leaf->isHierarchical(),
               leaf->getPlacementStatus().getString());

    // geometry info of dbInst
    if (leaf->getPlacementStatus().isPlaced()) {
      debugPrint(logger_,
                 utl::ODB,
                 "dumpDb",
                 1,
                 "geometry info of dbInst: (x:{}, y:{})",
                 leaf->getLocation().getX(),
                 leaf->getLocation().getY());
    }

    // mterm
    for (auto mterm : leaf->getMaster()->getMTerms()) {
      debugPrint(logger_,
                 utl::ODB,
                 "dumpDb",
                 1,
                 "\t dbMTerm {}, dir {}, get (mod)Net from iterm",
                 mterm->getName(),
                 mterm->getIoType().getString());
    }

    // leafInst level
    for (auto iTerm : leaf->getITerms()) {
      debugPrint(logger_,
                 utl::ODB,
                 "dumpDb",
                 1,
                 "\t dbITerm {}, dir {}, get (mod)Net from iterm",
                 iTerm->getName(),
                 iTerm->getIoType().getString());
      // get (mod)Net from iterm
      auto modNet = iTerm->getModNet();
      if (modNet) {
        debugPrint(logger_,
                   utl::ODB,
                   "dumpDb",
                   1,
                   "\t\t dbModNet {}",
                   modNet->getName());

        for (auto modBTerm : modNet->getModBTerms()) {
          debugPrint(logger_,
                     utl::ODB,
                     "dumpDb",
                     1,
                     "\t\t\t dbModBTerm {}, dir {}",
                     modBTerm->getName(),
                     modBTerm->getIoType().getString());
        }

        for (auto modITerm : modNet->getModITerms()) {
          debugPrint(logger_,
                     utl::ODB,
                     "dumpDb",
                     1,
                     "\t\t\t dbModITerm {}, of dbModInst {}",
                     modITerm->getName(),
                     modITerm->getParent()->getHierarchicalName());
        }

        for (auto bTerm : modNet->getBTerms()) {
          debugPrint(logger_,
                     utl::ODB,
                     "dumpDb",
                     1,
                     "\t\t\t dbBTerm {}",
                     bTerm->getName());
        }

        for (auto iTerm : modNet->getITerms()) {
          debugPrint(logger_,
                     utl::ODB,
                     "dumpDb",
                     1,
                     "\t\t\t dbITerm {}, dir {}",
                     iTerm->getName(),
                     iTerm->getIoType().getString());
        }
      }

      auto net = iTerm->getNet();
      if (net) {
        debugPrint(logger_,
                   utl::ODB,
                   "dumpDb",
                   1,
                   "\t\t dbNet {}, terms: {}, bterms: {}, iterms: {}, "
                   "drivingItermID: {}",
                   net->getName(),
                   net->getTermCount(),
                   net->getBTermCount(),
                   net->getITermCount(),
                   net->getDrivingITerm());
        auto firstITerm = net->get1stITerm();
        auto firstBTerm = net->get1stBTerm();

        if (firstITerm) {
          debugPrint(logger_,
                     utl::ODB,
                     "dumpDb",
                     1,
                     "\t\t\t 1stITerm {}, of dbInst {}",
                     firstITerm->getName(),
                     firstITerm->getInst()->getName());
        }

        if (firstBTerm) {
          debugPrint(logger_,
                     utl::ODB,
                     "dumpDb",
                     1,
                     "\t\t\t 1stBTerm {}",
                     firstBTerm->getName());

          // get bpin
          for (auto bPin : firstBTerm->getBPins()) {
            debugPrint(logger_,
                       utl::ODB,
                       "dumpDb",
                       1,
                       "\t\t\t\t bPin getPlacementStatus {}",
                       bPin->getPlacementStatus().getString());
          }
        }

        // dump net related bterms
        for (odb::dbBTerm* bterm : net->getBTerms()) {
          debugPrint(logger_,
                     utl::ODB,
                     "dumpDb",
                     1,
                     "\t\t\t bterm: {}",
                     bterm->getName());
          // dump bpin of bterm
          for (auto bPin : bterm->getBPins()) {
            debugPrint(logger_,
                       utl::ODB,
                       "dumpDb",
                       1,
                       "\t\t\t\t bPin getPlacementStatus {}",
                       bPin->getPlacementStatus().getString());
          }
        }
        // dump net related iterms
        for (odb::dbITerm* iterm : net->getITerms()) {
          debugPrint(logger_,
                     utl::ODB,
                     "dumpDb",
                     1,
                     "\t\t\t iterm {}, of dbInst {}",
                     iterm->getName(),
                     iterm->getInst()->getName());
        }
      }
    }
  }

  debugPrint(logger_, utl::ODB, "dumpDb", 1, "mod->getChildren()");
  for (auto child : mod->getChildren()) {
    debugPrint(logger_,
               utl::ODB,
               "dumpDb",
               1,
               "dbModInst in local hier {}, parent: {}",
               child->getHierarchicalName(),
               child->getParent()->getHierarchicalName());
    // dbModInst level
    for (auto modITerm : child->getModITerms()) {
      debugPrint(logger_,
                 utl::ODB,
                 "dumpDb",
                 1,
                 "\t dbModITerm {}, of dbModInst {}",
                 modITerm->getName(),
                 modITerm->getParent()->getHierarchicalName());
    }
    // recursivly dump dbModule
    dumpDbModule(child->getMaster(), hier + 1);
  }
}

void OpenRoad::dumpNetwork(sta::ConcreteNetwork* network)
{
  std::string group;
  std::string message;

  if (network == getVerilogNetwork()) {
    group = "dumpVerilogNetwork";
    message = "================VerilogNetwork================";
  } else if (network == getDbNetwork()) {
    group = "dumpDbNetwork";
    message = "================dbNetwork================";
  }

  debugPrint(logger_, utl::ODB, group.c_str(), 1, message.c_str());
  // statistic of VerilogNetwork

  if (network->topInstance()) {
    debugPrint(logger_,
               utl::ODB,
               group.c_str(),
               1,
               "inst: {}, pin: {}, net: {}, leafInst: {}, leafPin: {}",
               network->instanceCount(),
               network->pinCount(),
               network->netCount(),
               network->leafInstanceCount(),
               network->leafPinCount());

    auto libIt = network->libraryIterator();
    while (libIt->hasNext()) {
      auto lib = libIt->next();
      debugPrint(logger_,
                 utl::ODB,
                 group.c_str(),
                 1,
                 "library: {}",
                 network->name(lib));
    }

    auto libertyLibIt = network->libertyLibraryIterator();
    while (libertyLibIt->hasNext()) {
      auto lib = libertyLibIt->next();
      auto clib = reinterpret_cast<sta::ConcreteLibrary*>(lib);
      debugPrint(logger_,
                 utl::ODB,
                 group.c_str(),
                 1,
                 "libertyLibrary: {}",
                 clib->filename());
    }
  } else {
    logger_->warn(utl::ODB, 8000, "empty dbVerilogNetwork");
  }

  debugPrint(logger_, utl::ODB, group.c_str(), 1, message.c_str());

  dumpInstFromNetwork(network, network->topInstance(), 0, group);
}

void OpenRoad::dumpInstFromNetwork(sta::ConcreteNetwork* network,
                                   sta::Instance* inst,
                                   int hierLevel,
                                   std::string& group)
{
  if (inst == nullptr && hierLevel == 0) {
    logger_->warn(utl::ODB, 8001, "no instance in network");
    return;
  }
  if (inst == nullptr) {
    return;
  }
  std::string baseIndent;
  for (int i = 0; i < hierLevel; ++i) {
    baseIndent.append("\t");
  }
  // dump top instance
  if (hierLevel == 0) {
    debugPrint(logger_,
               utl::ODB,
               group.c_str(),
               1,
               "{}top instance: {}, cell: {}, hierLevel: {}, subInst: {}, pin: "
               "{}, net: {}",
               baseIndent,
               network->pathName(inst),
               network->cellName(inst),
               hierLevel,
               network->instanceCount(inst),
               network->pinCount(inst),
               network->netCount(inst));
    // pin
    dumpPinOfInstance(network, inst, hierLevel, group);
    // net
    dumpNetOfInstance(network, inst, hierLevel, group);
    // dump children
    auto childIt = network->childIterator(inst);
    while (childIt->hasNext()) {
      dumpInstFromNetwork(network, childIt->next(), hierLevel + 1, group);
    }
  } else {  // dump module instance or leaf instance
    debugPrint(logger_,
               utl::ODB,
               group.c_str(),
               1,
               "{} instance: {}, cell: {}, hierLevel: {}, subInst: {}, pin: "
               "{}, net: {}",
               baseIndent,
               network->pathName(inst),
               network->cellName(inst),
               hierLevel,
               network->instanceCount(inst),
               network->pinCount(inst),
               network->netCount(inst));
    // pin, pinNet, term, termNet
    dumpPinOfInstance(network, inst, hierLevel, group);
    // net
    dumpNetOfInstance(network, inst, hierLevel, group);
    // dump children
    auto childIt = network->childIterator(inst);
    while (childIt->hasNext()) {
      dumpInstFromNetwork(network, childIt->next(), hierLevel + 1, group);
    }
  }
}

void OpenRoad::dumpPinOfInstance(sta::ConcreteNetwork* network,
                                 sta::Instance* inst,
                                 int hierLevel,
                                 std::string& group)
{
  std::string baseIndent;
  for (int i = 0; i < hierLevel; ++i) {
    baseIndent.append("\t");
  }
  // port
  // cell
  auto cell = network->cell(inst);
  // cell->port
  auto portIt = network->portIterator(cell);
  while (portIt->hasNext()) {
    auto port = portIt->next();
    debugPrint(logger_,
               utl::ODB,
               group.c_str(),
               1,
               "{}\t port: {}, isBus: {}",
               baseIndent,
               network->name(port),
               network->isBus(port));
  }

  auto pinIt = network->pinIterator(inst);
  while (pinIt->hasNext()) {
    sta::Pin* pin = pinIt->next();
    debugPrint(logger_,
               utl::ODB,
               group.c_str(),
               1,
               "{}\t pin: {}, dir: {}",
               baseIndent,
               network->pathName(pin),
               network->direction(pin)->name());

    // get net from pin
    auto pinNet = network->net(pin);
    // pinNet may not excist
    if (pinNet) {
      debugPrint(logger_,
                 utl::ODB,
                 group.c_str(),
                 1,
                 "{}\t\t PinNet: {}",
                 baseIndent,
                 network->pathName(pinNet));
    } else {
      debugPrint(logger_,
                 utl::ODB,
                 group.c_str(),
                 1,
                 "{}\t\t pin {} does not connected to net",
                 baseIndent,
                 network->pathName(pin));
    }

    // get term from pin
    sta::Term* term = network->term(pin);
    // term may not exist
    if (term) {
      sta::Net* termNet = network->net(term);
      if (termNet) {
        debugPrint(logger_,
                   utl::ODB,
                   group.c_str(),
                   1,
                   "{}\t\t TermNet: {}",
                   baseIndent,
                   network->pathName(termNet));
      }
      // get pin from this term net
      auto termNetPinIt = network->pinIterator(termNet);
      while (termNetPinIt->hasNext()) {
        auto termNetPin = termNetPinIt->next();
        if (termNetPin) {
          debugPrint(logger_,
                     utl::ODB,
                     group.c_str(),
                     1,
                     "{}\t\t\t TermNetPin: {}",
                     baseIndent,
                     network->pathName(termNetPin));
        }
      }
    }
  }
}

void OpenRoad::dumpNetOfInstance(sta::ConcreteNetwork* network,
                                 sta::Instance* inst,
                                 int hierLevel,
                                 std::string& group)
{
  std::string baseIndent;
  for (int i = 0; i < hierLevel; ++i) {
    baseIndent.append("\t");
  }
  auto netIt = network->netIterator(inst);
  while (netIt->hasNext()) {
    sta::Net* net = netIt->next();
    debugPrint(logger_,
               utl::ODB,
               group.c_str(),
               1,
               "{}\t net: {}",
               baseIndent,
               network->pathName(net));
    // get pin from net
    auto pinIt = network->pinIterator(net);
    while (pinIt->hasNext()) {
      auto pin = pinIt->next();
      debugPrint(logger_,
                 utl::ODB,
                 group.c_str(),
                 1,
                 "{}\t\t pin: {}, dir: {}",
                 baseIndent,
                 network->pathName(pin),
                 network->direction(pin)->name());
    }
    // get term from net
    auto termIt = network->termIterator(net);
    while (termIt->hasNext()) {
      auto term = termIt->next();
      debugPrint(logger_,
                 utl::ODB,
                 group.c_str(),
                 1,
                 "{}\t\t term: {}",
                 baseIndent,
                 network->pathName(term));
    }
  }
}

void OpenRoad::dumpTechLibs()
{
  debugPrint(logger_,
             utl::ODB,
             "dumpPDK",
             1,
             "================dumpPDKs================");

  auto odb = getDb();

  // auto tech = odb->getTech();  // getTech() is obsolete in a multi-tech db
  // debugPrint(logger_,
  //            utl::ODB,
  //            "dumpPDK",
  //            1,
  //            "dbTech: {}, version:{}, layer: {}, routingLayer: {}, via:
  //            {}", tech->getName(), tech->getLefVersionStr(),
  //            tech->getLayerCount(),
  //            tech->getRoutingLayerCount(),
  //            tech->getViaCount());

  // Tech Level
  for (auto tech : odb->getTechs()) {
    debugPrint(logger_,
               utl::ODB,
               "dumpPDK",
               1,
               "dbTech: {}, version:{}, layer: {}, routingLayer: {}, via: {}, "
               "UnitsPerMicron: {}",
               tech->getName(),
               tech->getLefVersionStr(),
               tech->getLayerCount(),
               tech->getRoutingLayerCount(),
               tech->getViaCount(),
               tech->getDbUnitsPerMicron());
  }

  // Lib Level
  for (auto lib : odb->getLibs()) {
    debugPrint(logger_,
               utl::ODB,
               "dumpPDK",
               1,
               "dbLib: {}, getTech: {}, lefUnits: {}, master: {}",
               lib->getName(),
               lib->getTech()->getName(),
               lib->getLefUnits(),
               lib->getMasters().size());

    // lib -> master
    for (auto master : lib->getMasters()) {
      debugPrint(logger_,
                 utl::ODB,
                 "dumpPDK",
                 1,
                 "dbMaster: {}, Id: {}, area: {}, height: {}, width: {}, "
                 "isCoreAutoPlaceable: {}",
                 master->getName(),
                 master->getMasterId(),
                 master->getArea(),
                 master->getHeight(),
                 master->getWidth(),
                 master->getMTermCount(),
                 master->isCoreAutoPlaceable());
    }
  }

  debugPrint(logger_,
             utl::ODB,
             "dumpPDK",
             1,
             "================dumpPDKs================");
}

void OpenRoad::dumpRegionGroups()
{
  dbBlock* block = getDb()->getChip()->getBlock();
  for (odb::dbRegion* region : block->getRegions()) {
    debugPrint(logger_,
               utl::ODB,
               "dumpRegion",
               1,
               "dbRegion: {}, type: {}, regionInsts: {}",
               region->getName(),
               region->getRegionType().getString(),
               region->getRegionInsts().size());
    // instance in this region
    for (auto inst : region->getRegionInsts()) {
      debugPrint(
          logger_, utl::ODB, "dumpRegion", 1, "\tdbInst: {}", inst->getName());
    }
    // group in this region
    for (auto group : region->getGroups()) {
      debugPrint(logger_,
                 utl::ODB,
                 "dumpRegion",
                 1,
                 "dbGroup: {}, parentGroup: {}, region: {}",
                 group->getName(),
                 group->getParentGroup() ? group->getParentGroup()->getName()
                                         : "empty",
                 group->getRegion()->getName());
      // get mod instnce in group
      for (auto modInst : group->getModInsts()) {
        debugPrint(logger_,
                   utl::ODB,
                   "dumpRegion",
                   1,
                   "\tdbInst: {}",
                   modInst->getName());
      }
      // get instance in group
      for (auto inst : region->getRegionInsts()) {
        debugPrint(logger_,
                   utl::ODB,
                   "dumpRegion",
                   1,
                   "\tdbInst: {}",
                   inst->getName());
      }
    }
  }
}

}  // namespace ord
