#include <array>
#include <exception>

#include "gtest/gtest.h"
#include "helper.h"
#include "odb/db.h"

namespace odb {
namespace {

TEST_F(SimpleDbFixture, test_delete_module)
{
  createSimpleDB();

  auto and2 = db_->findMaster("and2");
  ASSERT_NE(and2, nullptr);

  auto inst = dbInst::create(db_->getChip()->getBlock(), and2, "i1");

  // Can't delete while there is an instance of the master
  EXPECT_THROW(dbMaster::destroy(and2), std::exception);

  dbInst::destroy(inst);
  dbMaster::destroy(and2);  // now we can

  // It is gone
  for (auto lib : db_->getLibs()) {
    for (auto master : lib->getMasters()) {
      EXPECT_NE(master, and2);
    }
  }
}

TEST_F(SimpleDbFixture, test_inst_creation_with_master_origin)
{
  createSimpleDB();
  auto lib = db_->findLib("lib1");
  auto block = db_->getChip()->getBlock();
  const std::array origins{
      Point(0, 0), Point(180, 0), Point(120, 80), Point(-120, -80)};

  for (const auto& origin : origins) {
    SCOPED_TRACE(::testing::Message()
                 << "master origin: " << origin.x() << ", " << origin.y());
    auto master = dbMaster::create(lib, "offset_master");
    master->setWidth(1000);
    master->setHeight(600);
    master->setOrigin(origin.x(), origin.y());
    master->setFrozen();

    auto inst = dbInst::create(block, master, "offset_inst");
    ASSERT_NE(inst, nullptr);
    const Rect expected_bbox(
        -origin.x(), -origin.y(), 1000 - origin.x(), 600 - origin.y());
    EXPECT_EQ(inst->getOrigin(), Point(0, 0));
    EXPECT_EQ(inst->getOrient(), dbOrientType::R0);
    EXPECT_EQ(inst->getPlacementStatus(), dbPlacementStatus::NONE);
    EXPECT_EQ(inst->getLocation(), Point(-origin.x(), -origin.y()));
    EXPECT_EQ(inst->getBBox()->getBox(), expected_bbox);

    // Reapplying the same origin must leave the bounding box unchanged.
    const Rect initial_bbox = inst->getBBox()->getBox();
    inst->setOrigin(0, 0);
    EXPECT_EQ(inst->getBBox()->getBox(), initial_bbox);

    // Setting the lower-left corner must account for the master origin.
    inst->setLocation(0, 0);
    EXPECT_EQ(inst->getOrigin(), origin);
    EXPECT_EQ(inst->getLocation(), Point(0, 0));
    EXPECT_EQ(inst->getBBox()->getBox(), Rect(0, 0, 1000, 600));

    dbInst::destroy(inst);
    dbMaster::destroy(master);
  }
}

}  // namespace
}  // namespace odb
