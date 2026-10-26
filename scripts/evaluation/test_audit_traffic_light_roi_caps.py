"""Pure ROI-cap audit tests; no ROS, GPU, bag or simulator."""
from audit_traffic_light_roi_caps import current_eligible, eligible_at_cap, summarize_packets


def packet(group=201326, padding=None, rejected=None, status='ready'):
    return dict(group_id=group, status=status, selector_config={'max_padding_px': 90},
                rois=[] if padding is None else [dict(head_id='C1256W000052', eligible=True,
                                                     padding_xy_px=padding)],
                rejected_heads=[] if rejected is None else [rejected])


def rejection(x=70., y=184.2, reason='projection_uncertainty_exceeds_bound'):
    return dict(head_id='C1256W000052', reason=reason, depth_m=8.76,
                required_padding_xy_px=[x, y])


def test_only_uncertainty_rejection_can_be_rescued_and_x90_stays_fixed():
    row = packet(rejected=rejection())
    assert not current_eligible(row)
    assert not eligible_at_cap(row, 180)
    assert eligible_at_cap(row, 200)
    assert not eligible_at_cap(packet(rejected=rejection(x=91.)), 240)
    assert not eligible_at_cap(packet(rejected=rejection(reason='housing_not_fully_in_image')), 240)
    assert not eligible_at_cap(packet(rejected=rejection(), status='missing'), 240)
    assert not eligible_at_cap(packet(group=0, rejected=rejection()), 240)


def test_existing_rois_are_retested_for_smaller_caps_not_unconditionally_trusted():
    row = packet(padding=[70., 190.])
    assert current_eligible(row)
    assert not eligible_at_cap(row, 180)
    assert eligible_at_cap(row, 200)


def test_missing_run_closes_on_group_change_and_stall_window_is_clipped():
    rows = [(0., packet(rejected=rejection())), (1., packet(rejected=rejection())),
            (2., packet(padding=[70., 170.])), (3., packet(group=0)),
            (20., packet(group=201351))]
    summary = summarize_packets(rows, stall_start_sec=.5)
    old = summary['missing_spans_by_y_cap']['180']['D_stall_window']
    assert old['longest_recorded_missing_span_sec'] == 1.5
    assert summary['missing_spans_by_y_cap']['180']['201326']['longest_recorded_missing_span_sec'] == 2.
    assert summary['missing_spans_by_y_cap']['200']['201326']['longest_recorded_missing_span_sec'] == 0.
    assert summary['packet_counts']['201326']['newly_eligible_y200'] == 2
