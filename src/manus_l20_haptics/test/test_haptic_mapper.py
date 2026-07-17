from manus_l20_haptics.haptic_mapper import HapticMapper, HapticMappingConfig


def test_mapper_scales_and_smooths_attack():
    mapper = HapticMapper(HapticMappingConfig(normal_force_full_scale=100.0, attack_alpha=0.5))

    assert mapper.update([100.0, 50.0, 0.0, -1.0, 200.0]) == [0.5, 0.25, 0.0, 0.0, 0.5]


def test_mapper_threshold_and_release():
    mapper = HapticMapper(
        HapticMappingConfig(
            normal_force_full_scale=100.0,
            attack_alpha=1.0,
            release_alpha=0.25,
            contact_threshold=0.2,
        )
    )

    assert mapper.update([100.0] * 5) == [1.0] * 5
    assert mapper.update([1.0] * 5) == [0.75] * 5


def test_mapper_can_use_approach_and_invert():
    mapper = HapticMapper(
        HapticMappingConfig(
            normal_force_full_scale=100.0,
            approach_full_scale=20.0,
            attack_alpha=1.0,
            use_approach=True,
            invert_fingers=True,
        )
    )

    assert mapper.update([0, 0, 0, 0, 0], [0, 5, 10, 15, 20]) == [1.0, 0.75, 0.5, 0.25, 0.0]
