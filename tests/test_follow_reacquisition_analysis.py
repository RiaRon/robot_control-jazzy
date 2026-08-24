from tools.analyze_follow_reacquisition import analyze


def test_aprime_follow_reacquisition_neighbourhood_is_well_conditioned():
    result = analyze(use_ros=False)
    lattice = result["offline_lattice"]

    assert result["tolerance_rad"] == 0.05
    assert lattice["samples"] == 3**7
    assert lattice["joint_limit_failures"] == 0
    assert lattice["minimum_joint_limit_margin_rad"] > 0.32
    assert lattice["minimum_jacobian_rank"] == 6
    assert lattice["minimum_sigma"] > 0.047
    assert lattice["maximum_condition_number"] < 39.0
