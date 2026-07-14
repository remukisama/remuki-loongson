#include "roundabout_controller.hpp"

#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* message)
{
    if (condition) return;
    std::cerr << "roundabout controller test failed: " << message << '\n';
    std::exit(1);
}

roundabout::Update step(
    roundabout::Controller& controller,
    long long now_ms,
    int inner_lower_missing,
    int outer_line_loss,
    bool exit_corner,
    bool exit_lane,
    int center_error = 0,
    bool inner_tow_found = true,
    float turn_yaw_deg = 0.0f,
    int inner_line_loss = 0)
{
    roundabout::Observation observation;
    observation.now_ms = now_ms;
    observation.inner_lower_missing = inner_lower_missing;
    observation.inner_line_loss = inner_line_loss;
    observation.outer_line_loss = outer_line_loss;
    observation.inner_tow_found = inner_tow_found;
    observation.turn_yaw_deg = turn_yaw_deg;
    observation.orbit_exit_corner = exit_corner;
    observation.exit_lane_visible = exit_lane;
    observation.center_error = center_error;
    return controller.update(observation);
}

void test_evidence_driven_left_ring()
{
    using roundabout::Phase;

    roundabout::Controller controller;
    controller.begin(roundabout::Side::Left, 1000);

    require(step(controller, 1060, 8, 0, false, false).phase == Phase::Approach,
            "approach must honor its minimum dwell time");
    step(controller, 1080, 8, 0, false, false);
    step(controller, 1090, 8, 0, false, false);
    step(controller, 1100, 8, 0, false, false);
    const auto passing = step(controller, 1110, 8, 0, false, false);
    require(passing.changed && passing.phase == Phase::PassOpening,
            "four approach-edge frames must start passing the first opening");

    step(controller, 1120, 6, 0, false, false);
    require(step(controller, 1390, 0, 0, false, false).phase ==
                Phase::PassOpening,
            "opening recovery before the minimum pass time must be ignored");
    step(controller, 1410, 0, 0, false, false);
    step(controller, 1420, 0, 0, false, false);
    step(controller, 1430, 0, 0, false, false);
    const auto seeking = step(controller, 1440, 0, 0, false, false);
    require(seeking.changed && seeking.phase == Phase::SeekEntrance,
            "clearing the first opening must only start seeking the entrance");

    require(step(controller, 1670, 0, 0, false, false).phase ==
                Phase::SeekEntrance,
            "a visible inner boundary must keep the first-opening repair active");
    step(controller, 1680, 0, 0, false, false, 0, false);
    step(controller, 1690, 0, 0, false, false, 0, false);
    step(controller, 1700, 0, 0, false, false, 0, false);
    require(step(controller, 1710, 0, 0, false, false, 0, false).phase ==
                Phase::SeekEntrance,
            "a far-only entrance opening must not cancel the repair line");
    step(controller, 1720, 0, 0, false, false, 0, false, 0.0f, 24);
    step(controller, 1730, 0, 0, false, false, 0, false, 0.0f, 24);
    step(controller, 1740, 0, 0, false, false, 0, false, 0.0f, 24);
    const auto entered = step(controller, 1750, 0, 0, false, false, 0, false, 0.0f, 24);
    require(entered.changed && entered.phase == Phase::Enter,
            "a stable middle-distance entrance opening must start entry");

    step(controller, 1760, 0, 11, false, false);
    step(controller, 1770, 0, 11, false, false);
    step(controller, 1780, 0, 11, false, false);
    const auto recovering = step(controller, 1790, 0, 11, false, false);
    require(recovering.changed && recovering.phase == Phase::EnterRecover,
            "outer boundary loss must mark the entry tangent crossing");

    step(controller, 1800, 0, 2, false, false);
    step(controller, 1810, 0, 2, false, false);
    step(controller, 1820, 0, 2, false, false);
    const auto orbiting = step(controller, 1830, 0, 2, false, false);
    require(orbiting.changed && orbiting.phase == Phase::Orbit,
            "outer boundary recovery must start orbit tracking");

    require(step(controller, 2720, 0, 20, true, false, 0, true, 285.0f).phase == Phase::Orbit,
            "exit geometry before the minimum orbit time must be ignored");
    require(step(controller, 2730, 0, 20, true, false, 0, true, 285.0f).phase == Phase::Orbit,
            "an in-ring corner before the minimum exit yaw must be ignored");
    step(controller, 2740, 0, 8, false, true, 0, true, 295.0f);
    step(controller, 2750, 0, 8, false, true, 0, true, 295.0f);
    step(controller, 2760, 0, 8, false, true, 0, true, 295.0f);
    require(step(controller, 2770, 0, 8, false, true, 0, true, 295.0f).phase == Phase::Orbit,
            "a recovered outer boundary alone must not end the ring curve");
    step(controller, 2780, 0, 20, true, false, 0, true, 330.0f);
    step(controller, 2790, 0, 20, true, false, 0, true, 330.0f);
    step(controller, 2800, 0, 20, true, false, 0, true, 330.0f);
    const auto exiting = step(controller, 2810, 0, 20, true, false, 0, true, 330.0f);
    require(exiting.changed && exiting.phase == Phase::Exit,
            "the completed 330-degree orbit gate must start exit steering");

    require(step(controller, 3020, 0, 0, false, true).phase == Phase::Exit,
            "the exit lane must not release reverse steering too early");
    step(controller, 3030, 0, 0, false, true);
    step(controller, 3040, 0, 0, false, true);
    step(controller, 3050, 0, 0, false, true);
    const auto aligning = step(controller, 3060, 0, 0, false, true);
    require(aligning.changed && aligning.phase == Phase::Align,
            "stable exit lane evidence must start alignment");

    require(!step(controller, 3370, 0, 0, false, true, 0, true,
                  0.0f, 23).finished,
            "alignment must honor its minimum dwell time");
    step(controller, 3380, 0, 0, false, true, 0, true, 0.0f, 23);
    step(controller, 3390, 0, 0, false, true, 0, true, 0.0f, 23);
    step(controller, 3400, 0, 0, false, true, 0, true, 0.0f, 23);
    require(!step(controller, 3410, 0, 0, false, true, 0, true,
                  0.0f, 23).finished,
            "center error alone must not release a partially open exit lane");
    step(controller, 3420, 0, 0, false, true, 0, true, 0.0f, 10);
    step(controller, 3430, 0, 0, false, true, 0, true, 0.0f, 10);
    step(controller, 3440, 0, 0, false, true, 0, true, 0.0f, 10);
    const auto finished = step(controller, 3450, 0, 0, false, true, 0, true,
                               0.0f, 10);
    require(finished.finished && finished.phase == Phase::Align,
            "four centered exit frames must finish the ring");
}

void test_timeout_fallbacks_and_confirmation_reset()
{
    using roundabout::Phase;

    roundabout::Controller controller;
    controller.begin(roundabout::Side::Right, 0);

    // A single early edge must not leak into the later confirmation window.
    step(controller, 80, 8, 0, false, false);
    step(controller, 90, 0, 0, false, false);
    step(controller, 400, 0, 0, false, false);
    step(controller, 410, 0, 0, false, false);
    step(controller, 420, 0, 0, false, false);
    const auto passing = step(controller, 430, 0, 0, false, false);
    require(passing.changed && passing.phase == Phase::PassOpening,
            "approach timeout must only start passing the opening");

    step(controller, 1530, 0, 0, false, false);
    step(controller, 1540, 0, 0, false, false);
    step(controller, 1550, 0, 0, false, false);
    const auto seeking = step(controller, 1560, 0, 0, false, false);
    require(seeking.changed && seeking.phase == Phase::SeekEntrance,
            "opening pass timeout must seek the entrance instead of turning");

    require(step(controller, 2850, 0, 0, false, false).phase ==
                Phase::SeekEntrance,
            "missing entrance evidence must not abort before the dwell time");
    step(controller, 2860, 0, 0, false, false);
    step(controller, 2870, 0, 0, false, false);
    step(controller, 2880, 0, 0, false, false);
    const auto aborted = step(controller, 2890, 0, 0, false, false);
    require(aborted.aborted && aborted.phase == Phase::SeekEntrance &&
                !controller.active(),
            "missing entrance evidence must abort instead of forcing a turn");

    // A new detection with real second-opening evidence must still enter.
    controller.begin(roundabout::Side::Right, 0);
    step(controller, 80, 8, 0, false, false);
    step(controller, 90, 8, 0, false, false);
    step(controller, 100, 8, 0, false, false);
    step(controller, 110, 8, 0, false, false);
    step(controller, 120, 6, 0, false, false);
    step(controller, 410, 0, 0, false, false);
    step(controller, 420, 0, 0, false, false);
    step(controller, 430, 0, 0, false, false);
    step(controller, 440, 0, 0, false, false);
    step(controller, 660, 4, 0, false, false, 0, false);
    step(controller, 670, 4, 0, false, false, 0, false);
    step(controller, 680, 4, 0, false, false, 0, false);
    const auto entered = step(controller, 690, 4, 0, false, false, 0, false);
    require(entered.changed && entered.phase == Phase::Enter,
            "real entrance evidence must still start the turn");

    step(controller, 2190, 0, 0, false, false);
    step(controller, 2200, 0, 0, false, false);
    step(controller, 2210, 0, 0, false, false);
    const auto recovering = step(controller, 2220, 0, 0, false, false);
    require(recovering.changed && recovering.phase == Phase::EnterRecover,
            "entry must have a bounded loss-evidence timeout");

    step(controller, 3420, 0, 20, false, false);
    step(controller, 3430, 0, 20, false, false);
    step(controller, 3440, 0, 20, false, false);
    const auto orbiting = step(controller, 3450, 0, 20, false, false);
    require(orbiting.changed && orbiting.phase == Phase::Orbit,
            "entry recovery must have a bounded timeout");

    require(step(controller, 4250, 0, 20, true, false, 0, true, 110.0f).phase ==
                Phase::Orbit,
            "an early ring corner must be ignored before enough accumulated yaw");
    require(step(controller, 6800, 0, 0, false, false, 0, true, 335.0f).phase ==
                Phase::Orbit,
            "330-degree yaw without exit geometry must remain in the ring");
    step(controller, 6810, 0, 0, false, false, 0, true, 380.0f);
    step(controller, 6820, 0, 0, false, false, 0, true, 380.0f);
    step(controller, 6830, 0, 0, false, false, 0, true, 380.0f);
    const auto exiting = step(controller, 6840, 0, 0, false, false, 0, true, 380.0f);
    require(exiting.changed && exiting.phase == Phase::Exit,
            "accumulated yaw fallback must still reach exit search");

    step(controller, 8040, 0, 20, false, false);
    step(controller, 8050, 0, 20, false, false);
    step(controller, 8060, 0, 20, false, false);
    const auto aligning = step(controller, 8070, 0, 20, false, false);
    require(aligning.changed && aligning.phase == Phase::Align,
            "exit search must have a bounded fallback");
    require(!step(controller, 8980, 0, 20, false, false, 55).finished,
            "alignment timeout with a fully lost lane must not report ring completion");
    step(controller, 8990, 0, 0, false, true, 20, true, 0.0f, 0);
    step(controller, 9000, 0, 0, false, true, 20, true, 0.0f, 0);
    step(controller, 9010, 0, 0, false, true, 20, true, 0.0f, 0);
    const auto recovered =
        step(controller, 9020, 0, 0, false, true, 20, true, 0.0f, 0);
    require(recovered.finished && recovered.phase == Phase::Align,
            "a fully recovered lane must leave alignment after its timeout");
}

void test_evidence_entry_and_stale_yaw_exit()
{
    using roundabout::Phase;

    roundabout::Controller controller;
    controller.begin(roundabout::Side::Left, 0);

    step(controller, 80, 8, 0, false, false);
    step(controller, 90, 8, 0, false, false);
    step(controller, 100, 8, 0, false, false);
    step(controller, 110, 8, 0, false, false);
    step(controller, 120, 6, 0, false, false);
    step(controller, 410, 0, 0, false, false);
    step(controller, 420, 0, 0, false, false);
    step(controller, 430, 0, 0, false, false);
    step(controller, 440, 0, 0, false, false);
    step(controller, 660, 4, 0, false, false, 0, false);
    step(controller, 670, 4, 0, false, false, 0, false);
    step(controller, 680, 4, 0, false, false, 0, false);
    step(controller, 690, 4, 0, false, false, 0, false);
    step(controller, 700, 0, 11, false, false);
    step(controller, 710, 0, 11, false, false);
    step(controller, 720, 0, 11, false, false);
    step(controller, 730, 0, 11, false, false);
    step(controller, 740, 0, 2, false, false);
    step(controller, 750, 0, 2, false, false);
    step(controller, 760, 0, 2, false, false);
    step(controller, 770, 0, 2, false, false);

    require(step(controller, 1670, 0, 0, false, true, 0, true,
                 335.0f, 0).phase == Phase::Orbit,
            "an intact inner boundary must reject a false 330-degree exit");
    require(step(controller, 6760, 0, 0, false, true, 0, true,
                 280.0f, 12).phase == Phase::Orbit,
            "exit geometry before six seconds must not bypass a stale yaw gate");
    step(controller, 6780, 0, 0, false, true, 0, true, 280.0f, 12);
    step(controller, 6790, 0, 0, false, true, 0, true, 280.0f, 12);
    step(controller, 6800, 0, 0, false, true, 0, true, 280.0f, 12);
    const auto exiting = step(controller, 6810, 0, 0, false, true, 0, true,
                              280.0f, 12);
    require(exiting.changed && exiting.phase == Phase::Exit,
            "a tracked lane plus inner opening must exit before the yaw fallback");
}

}  // namespace

int main()
{
    test_evidence_driven_left_ring();
    test_timeout_fallbacks_and_confirmation_reset();
    test_evidence_entry_and_stale_yaw_exit();
    std::cout << "roundabout controller tests passed\n";
    return 0;
}
