"""Passive ordered-AB reconstruction and offline decoder comparisons."""
from collections import Counter
FORWARD={1,7,14,8}
REVERSE={2,11,13,4}

class TraceCollector:
    def __init__(self,axis):
        self.axis=axis;self.last_count=None;self.gaps=[];self.restarts=0
    def feed(self,row):
        if row['axis']!=self.axis:return []
        count=row['event_count']
        if self.last_count is None:self.last_count=count;return []
        delta=(count-self.last_count)&0xffffffff
        if delta>=0x80000000:
            self.restarts+=1;self.last_count=count;return []
        if not delta:return []
        available=min(delta,len(row['transitions']))
        if available<delta:self.gaps.append(dict(after=self.last_count,missing=delta-available))
        events=[((count-available+i+1)&0xffffffff,value)
                for i,value in enumerate(row['transitions'][-available:])]
        self.last_count=count
        return events

def replay(events):
    totals=dict(x4_valid_only=0,a_scaled2=0,b_scaled2=0,experimental_last_legal_direction=0)
    direction=0;discontinuous=0;diagonal=0;last=None;transitions=Counter();triples=Counter();history=[]
    for event_id,value in events:
        old,current=value>>2,value&3
        if last is not None and old!=last:discontinuous+=1;direction=0;history=[]
        last=current;transitions['{:02b}->{:02b}'.format(old,current)]+=1
        history.append(value)
        if len(history)>=3:triples[' '.join('%X'%v for v in history[-3:])]+=1
        legal=1 if value in FORWARD else -1 if value in REVERSE else 0
        totals['x4_valid_only']+=legal
        if (old^current)&2:totals['a_scaled2']+=2 if (current>>1)==(current&1) else -2
        if (old^current)&1:totals['b_scaled2']+=2 if (current>>1)!=(current&1) else -2
        if legal:
            direction=legal;totals['experimental_last_legal_direction']+=legal
        elif (old^current)==3:
            diagonal+=1;totals['experimental_last_legal_direction']+=2*direction
    return dict(events=len(events),totals=totals,diagonal=diagonal,chain_discontinuities=discontinuous,
                transition_counts=dict(transitions),common_triples=triples.most_common(12),
                scope='Offline hypotheses only; no physical ground truth, no motor control or command-sign substitution')
