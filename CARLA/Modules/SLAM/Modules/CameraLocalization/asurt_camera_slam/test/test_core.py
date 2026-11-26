"""Synthetic world landmarks test geometric observability, never actor truth."""
import math

import numpy as np

from asurt_camera_slam.core import (
    CameraMapLocalizer, Observation, Pose2, transform)


# Simulated fixed cones in a planar odometry-anchored local map.
WORLD = [
    (5., 2., 0), (6., -1.8, 1), (3.3, 1.1, 2),
    (7.5, -2.2, 0), (9., 1.6, 1), (4.2, -1.1, 3)
]


def observe(pose, world=WORLD, confidence=.95):
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    output = []
    for x, y, color in world:
        dx, dy = x-pose.x, y-pose.y
        output.append(Observation(c*dx+s*dy, -s*dx+c*dy,
                                  -.25, color, confidence))
    return output


def warm(estimator, world=WORLD, stationary=False, origin=(0., 0.)):
    for k in range(6):
        x = 0. if stationary else .20*k
        pose = Pose2(origin[0]+x, origin[1], 0.)
        estimator.step(pose, observe(pose, world), 1.+.1*k)


def error(pose, truth):
    return math.hypot(pose.x-truth.x, pose.y-truth.y) + abs(pose.yaw-truth.yaw)


def test_corrects_metric_planar_odometry_drift_using_three_plus_cones():
    estimator = CameraMapLocalizer()
    warm(estimator)
    assert len(estimator.confirmed()) >= 4
    truth = Pose2(1.2, 0., 0.)
    drifted_odom = Pose2(1.42, .12, .065)
    report = estimator.step(drifted_odom, observe(truth), 1.6)
    assert report.state == 'corrected'
    assert report.pose is not None and report.inliers >= 3
    assert error(report.pose, truth) < error(drifted_odom, truth)*.85
    assert report.residual_rms_m < .30
    assert report.innovation_m > .01
    assert np.all(np.linalg.eigvalsh(report.covariance) > 0.)
    assert report.covariance[2, 2] >= .025**2


def test_large_negative_silverstone_coordinates_and_flatasphalt_origin_agree():
    """Translations of the whole map must not change relative SLAM geometry."""
    results = []
    for origin in ((0., 0.), (-384.3957, 234.7532)):
        landmarks = [(x+origin[0], y+origin[1], color)
                     for x, y, color in WORLD]
        solver = CameraMapLocalizer()
        warm(solver, landmarks, origin=origin)
        truth = Pose2(origin[0]+1.2, origin[1], 0.)
        odom = Pose2(truth.x+.22, truth.y+.12, .065)
        fit = solver.step(odom, observe(truth, landmarks), 1.6)
        assert fit.state == 'corrected'
        assert error(fit.pose, truth) < error(odom, truth)*.85
        assert np.all(np.linalg.eigvalsh(fit.covariance) > 0)
        results.append((
            fit.pose.x-origin[0], fit.pose.y-origin[1], fit.pose.yaw))
    assert np.allclose(results[0], results[1], atol=1e-7)


def test_only_two_cones_never_publish_pose_even_after_repeated_motion():
    estimator = CameraMapLocalizer()
    warm(estimator, WORLD[:2])
    result = estimator.step(Pose2(1.4, .1, .04),
                            observe(Pose2(1.2, 0., 0.), WORLD[:2]), 1.6)
    assert result.state == 'insufficient_repeated_cones'
    assert result.pose is None


def test_stationary_repeated_frames_do_not_confirm_correlated_views():
    estimator = CameraMapLocalizer()
    warm(estimator, stationary=True)
    assert len(estimator.confirmed()) == 0
    result = estimator.step(Pose2(.2, 0., 0.),
                            observe(Pose2(0., 0., 0.)), 1.6)
    assert result.pose is None


def test_collinear_cones_rejected_as_degenerate_geometry():
    estimator = CameraMapLocalizer()
    collinear = [(5., 0., 0), (7., 0., 1), (9., 0., 2), (11., 0., 3)]
    warm(estimator, collinear)
    result = estimator.step(Pose2(1.4, .1, .04),
                            observe(Pose2(1.2, 0., 0.), collinear), 1.6)
    assert result.state == 'degenerate_landmark_geometry'
    assert result.pose is None


def test_color_and_confidence_mismatch_cannot_form_pose():
    estimator = CameraMapLocalizer()
    warm(estimator)
    strange = [Observation(d.x, d.y, d.z, (d.color+1)%4, .2)
               for d in observe(Pose2(1.2, 0., 0.))]
    result = estimator.step(Pose2(1.4, .1, 0.), strange, 1.6)
    assert result.pose is None
    assert result.accepted_observations == 0
    assert len(estimator.confirmed()) >= 4


def test_mahalanobis_gate_and_color_reject_plausible_false_matches():
    solver = CameraMapLocalizer()
    warm(solver)
    anchor = next(a for a in solver.confirmed() if a.color == 0)
    first = observe(Pose2(1., 0., 0.))[0]
    # 1.6 m is inside the hard 1.7 m radius but outside the uncertainty gate.
    shifted = Observation(first.x+1.6, first.y, first.z, 0, .95)
    wrong_color = Observation(first.x, first.y, first.z, 1, .95)
    assert solver._associate([shifted], Pose2(1., 0., 0.), [anchor]) == []
    assert solver._associate([wrong_color], Pose2(1., 0., 0.), [anchor]) == []
    assert len(solver._associate([first], Pose2(1., 0., 0.), [anchor])) == 1


def test_five_consistent_cones_with_one_spurious_cone_still_correct():
    solver = CameraMapLocalizer()
    warm(solver)
    truth = Pose2(1.2, 0., 0.)
    dets = observe(truth)
    false = dets[0]
    dets[0] = Observation(false.x+1.5, false.y+.9,
                          false.z, false.color, false.confidence)
    prior = Pose2(1.42, .12, .065)
    result = solver.step(prior, dets, 1.6)
    assert result.state == 'corrected'
    assert result.inliers >= 3
    assert error(result.pose, truth) < error(prior, truth)


def test_rejected_large_teleport_and_time_rewind_clear_stable_map():
    estimator = CameraMapLocalizer()
    warm(estimator)
    assert estimator.confirmed()
    teleported = Pose2(100., 0., 0.)
    report = estimator.step(teleported, observe(teleported), 1.6)
    assert report.reset_reason == 'odom_teleport_or_world_change'
    assert report.pose is None
    assert not estimator.confirmed()
    warm_again = CameraMapLocalizer()
    warm(warm_again)
    rewind = warm_again.step(Pose2(0., 0., 0.), observe(Pose2(0., 0., 0.)), .5)
    assert rewind.reset_reason == 'time_rewind'
    assert not warm_again.confirmed()


def test_bounded_map_and_finite_covariance_after_sequential_corrections():
    estimator = CameraMapLocalizer(max_anchors=18, max_detections=8)
    warm(estimator)
    for k in range(6, 18):
        truth = Pose2(.20*k, 0., 0.)
        odom = Pose2(.20*k+.10, .05, .02)
        result = estimator.step(odom, observe(truth), 1.+.1*k)
        assert len(estimator.anchors) <= 18
        assert np.all(np.linalg.eigvalsh(estimator.covariance) > 0.)
        if result.pose is not None:
            assert result.state == 'corrected'
            assert np.all(np.isfinite(result.covariance))
    short = Observation(4., 1., -.2, 0, .9)
    distant = Observation(12., 1., -.2, 0, .9)
    assert estimator._variance(distant) > 9*estimator._variance(short)


def test_repeated_updates_freeze_confirmed_anchor_positions():
    estimator = CameraMapLocalizer()
    warm(estimator)
    original = {a.uid: a.xy.copy() for a in estimator.confirmed()}
    estimator.step(Pose2(1.35, .06, 0.), observe(Pose2(1.2, 0., 0.)), 1.6)
    for a in estimator.confirmed():
        assert np.allclose(a.xy, original[a.uid], atol=1e-12)
