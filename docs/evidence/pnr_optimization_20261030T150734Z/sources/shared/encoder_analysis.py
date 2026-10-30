"""Read-only cumulative-counter differences; no inferred physical CPR or RPM."""
COUNTERS=('sync_a_edges','sync_b_edges','filtered_a_edges','filtered_b_edges',
          'a_positive','a_negative','b_positive','b_negative','diagonal')

def difference(before,after):
    if before['axis']!=after['axis'] or before.get('mode')!=after.get('mode'):
        raise ValueError('Different diagnostic channels or modes')
    if ((after['sequence']-before['sequence'])&65535)>32767:
        raise ValueError('Diagnostic sequence restarted')
    delta={key:(after[key]-before[key])&0xffffffff for key in COUNTERS}
    # Reject apparent reset deltas instead of presenting them as huge motion.
    if any(value>10000000 for value in delta.values()):raise ValueError('Counter reset or unsupported capture interval')
    for phase in ('a','b'):
        plus,minus=delta[phase+'_positive'],delta[phase+'_negative']
        total=plus+minus
        delta[phase+'_scaled_net']=2*(plus-minus)
        delta[phase+'_cancellation_fraction']=1-abs(plus-minus)/total if total else None
    return delta

def summarize_rows(rows):
    result={}
    for axis in ('1','2'):
        observations=[row['encoder_diagnostics'][axis] for row in rows
                      if axis in row.get('encoder_diagnostics',{}) and row['encoder_diagnostics'][axis].get('age_ms',float('inf'))<600]
        if len(observations)<2:
            result[axis]=dict(available=False,reason='insufficient fresh diagnostic snapshots');continue
        try:result[axis]=dict(available=True,delta=difference(observations[0],observations[-1]),
                            first_sequence=observations[0]['sequence'],last_sequence=observations[-1]['sequence'])
        except ValueError as exc:result[axis]=dict(available=False,reason=str(exc))
    return result
