"""Kinematics tested against closed forms, not against itself.

Gravity torque is what will be sent to the motors as feedforward, so a sign
error here accelerates the arm rather than merely misposing it. Every case
below has an answer derivable by hand.
"""

import numpy as np
import pytest

from robot_control.kinematics import (
    Chain,
    KinematicsError,
    Link,
    Revolute,
    chain_from_urdf,
)

G = 9.80665


def _pendulum(length=1.0, mass=2.0, axis=(0.0, 1.0, 0.0)) -> Chain:
    """One link rotating about y at the origin, centre of mass at +x * length.

    Rotating about +y swings the mass in the xz plane, so gravity torque is a
    plain m*g*L*cos(theta) with theta measured from +x.
    """
    return Chain(
        (
            Revolute(
                name="j1",
                axis=np.array(axis, dtype=float),
                origin=np.zeros(3),
                rotation=np.eye(3),
                child="l1",
            ),
        ),
        (Link(name="l1", mass=mass, com=np.array([length, 0.0, 0.0])),),
    )


def test_pendulum_gravity_torque_matches_the_closed_form():
    chain = _pendulum(length=1.0, mass=2.0)

    for theta in (0.0, 0.3, 1.0, np.pi / 2, -0.7):
        # Positive rotation about +y tips +x down towards -z, so the gravity
        # torque about the joint opposes it with the same sign convention.
        expected = -2.0 * G * 1.0 * np.cos(theta)
        got = chain.gravity_torque(np.array([theta]))
        np.testing.assert_allclose(got, [expected], atol=1e-9)


def test_pendulum_hanging_straight_down_needs_no_torque():
    """Straight down is equilibrium; anything else is a sign error."""
    chain = _pendulum()

    np.testing.assert_allclose(
        chain.gravity_torque(np.array([np.pi / 2])), [0.0], atol=1e-9
    )
    np.testing.assert_allclose(
        chain.gravity_torque(np.array([-np.pi / 2])), [0.0], atol=1e-9
    )


def test_gravity_torque_scales_with_mass_and_length():
    light = _pendulum(length=1.0, mass=1.0).gravity_torque(np.array([0.0]))
    heavy = _pendulum(length=1.0, mass=3.0).gravity_torque(np.array([0.0]))
    far = _pendulum(length=2.0, mass=1.0).gravity_torque(np.array([0.0]))

    np.testing.assert_allclose(heavy, light * 3.0)
    np.testing.assert_allclose(far, light * 2.0)


def test_a_joint_parallel_to_gravity_carries_no_gravity_torque():
    """A yaw joint about z cannot lift or lower its own load."""
    chain = _pendulum(axis=(0.0, 0.0, 1.0))

    for theta in (0.0, 0.5, 2.0):
        np.testing.assert_allclose(
            chain.gravity_torque(np.array([theta])), [0.0], atol=1e-9
        )


def _two_link() -> Chain:
    """Two y-axis joints in a row, each link 1 m long with 1 kg at its tip."""
    return Chain(
        (
            Revolute("j1", np.array([0.0, 1.0, 0.0]), np.zeros(3), np.eye(3), "l1"),
            Revolute(
                "j2", np.array([0.0, 1.0, 0.0]), np.array([1.0, 0.0, 0.0]), np.eye(3), "l2"
            ),
        ),
        (
            Link("l1", 1.0, np.array([1.0, 0.0, 0.0])),
            Link("l2", 1.0, np.array([1.0, 0.0, 0.0])),
        ),
        # The tip sits at the far end of the second link, a metre past its joint.
        tip=np.array(
            [
                [1.0, 0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    )


def test_two_link_gravity_torque_matches_the_closed_form():
    chain = _two_link()
    # Fully extended along +x: joint 1 carries both masses at 1 m and 2 m,
    # joint 2 carries only the outer mass at 1 m.
    np.testing.assert_allclose(
        chain.gravity_torque(np.zeros(2)),
        [-G * (1.0 * 1.0 + 1.0 * 2.0), -G * 1.0 * 1.0],
        atol=1e-9,
    )


def test_forward_kinematics_places_the_tip():
    chain = _two_link()

    np.testing.assert_allclose(chain.pose(np.zeros(2))[:3, 3], [2.0, 0.0, 0.0])
    # Folding the second joint by 90 degrees about +y drops the tip to -z.
    np.testing.assert_allclose(
        chain.pose(np.array([0.0, np.pi / 2]))[:3, 3], [1.0, 0.0, -1.0], atol=1e-9
    )


def test_jacobian_matches_a_finite_difference_of_forward_kinematics():
    """The servo loop inverts this Jacobian, so it must be the real derivative."""
    chain = _two_link()
    q = np.array([0.3, -0.6])
    step = 1e-7

    jacobian = chain.jacobian(q)
    for column in range(len(q)):
        nudged = q.copy()
        nudged[column] += step
        difference = (chain.pose(nudged)[:3, 3] - chain.pose(q)[:3, 3]) / step
        np.testing.assert_allclose(jacobian[:3, column], difference, atol=1e-5)


def test_chain_rejects_a_joint_count_mismatch():
    chain = _two_link()

    with pytest.raises(KinematicsError, match="2 joints"):
        chain.pose(np.array([0.1]))


URDF = """<?xml version="1.0"?>
<robot name="fixture">
  <link name="base"/>
  <link name="l1">
    <inertial>
      <origin xyz="0.5 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="l1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="50" velocity="2"/>
  </joint>
  <joint name="fixed_tip" type="fixed">
    <parent link="l1"/>
    <child link="tip"/>
    <origin xyz="1.0 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_chain_from_urdf_reads_the_chain_to_a_tip_link(tmp_path):
    path = tmp_path / "fixture.urdf"
    path.write_text(URDF)

    chain = chain_from_urdf(path.read_text(), ["j1"], "tip")

    assert [joint.name for joint in chain.joints] == ["j1"]
    # The fixed joint to the tip is folded in, not treated as a degree of freedom.
    np.testing.assert_allclose(chain.pose(np.zeros(1))[:3, 3], [1.0, 0.0, 0.1])
    # Gravity torque uses the inertial origin, 0.5 m out, not the link origin.
    np.testing.assert_allclose(
        chain.gravity_torque(np.zeros(1)), [-2.0 * G * 0.5], atol=1e-9
    )


def test_chain_from_urdf_lumps_mass_bolted_on_past_the_last_joint():
    """The hand and tool centre point hang off the last joint by fixed joints.

    Left out, that mass loads nothing and gravity torque comes out low — which
    is exactly the direction the first live comparison was wrong in.
    """
    with_hand = URDF.replace(
        '  <link name="tip"/>',
        """  <link name="tip">
    <inertial>
      <origin xyz="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>""",
    )

    chain = chain_from_urdf(with_hand, ["j1"], "tip")

    # 2 kg at 0.5 m plus 1 kg at 1.0 m: 3 kg centred at (2*0.5 + 1*1.0)/3.
    link = chain.links[0]
    assert link.mass == pytest.approx(3.0)
    np.testing.assert_allclose(link.com, [2.0 / 3.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(
        chain.gravity_torque(np.zeros(1)), [-3.0 * G * (2.0 / 3.0)], atol=1e-9
    )


def test_chain_from_urdf_ignores_ros2_control_joint_elements():
    """A ros2_control block names the same joints, with no parent or child.

    Searching the tree recursively lets those shadow the kinematic joints, and
    the chain then appears not to reach its own tip.
    """
    shadowed = URDF.replace(
        "</robot>",
        """
  <ros2_control name="fixture_hardware" type="system">
    <hardware><plugin>mock_components/GenericSystem</plugin></hardware>
    <joint name="j1">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>""",
    )

    chain = chain_from_urdf(shadowed, ["j1"], "tip")

    np.testing.assert_allclose(chain.pose(np.zeros(1))[:3, 3], [1.0, 0.0, 0.1])


def test_chain_from_urdf_rejects_an_unknown_joint(tmp_path):
    with pytest.raises(KinematicsError, match="j9"):
        chain_from_urdf(URDF, ["j9"], "tip")


def test_chain_from_urdf_rejects_an_unreachable_tip(tmp_path):
    with pytest.raises(KinematicsError, match="nowhere"):
        chain_from_urdf(URDF, ["j1"], "nowhere")
