import bisect, json, math, statistics, sys
from pathlib import Path
p = Path(sys.argv[1])
d = json.loads(p.read_text())
h = d['history']
prefix = '/AdvantageKit/'
states = h.get(prefix+'RealOutputs/Intake/TargetState', [])
result = {'capture_utc':d['capture_utc'], 'duration_sec':round(d['duration_sec'],2), 'holds':[]}
for a, b in zip(states, states[1:]):
    if a[2] != 'kIntaking': continue
    lo, hi = a[1], b[1]
    row = {'duration_sec':round((hi-lo)/1e6,3)}
    for suffix in ['AppliedVolts','StatorCurrent','BusVolts','Angle']:
        samples = h[prefix+'AngularSubsystems/IntakePivot/'+suffix]
        index = bisect.bisect_right([x[1] for x in samples],lo)-1
        values = ([samples[index][2]] if index>=0 else []) + [x[2] for x in samples if lo<=x[1]<hi]
        if suffix=='Angle': values=[math.degrees(x) for x in values]
        row[suffix] = {'min':round(min(values),3),'median':round(statistics.median(values),3),'max':round(max(values),3)}
    result['holds'].append(row)
for suffix in ['ConfigError','ConfigReady','ConfigurationGeneration']:
    result[suffix] = h.get(prefix+'AngularSubsystems/IntakePivot/'+suffix,[])
result['PositionCommandResult'] = h.get(prefix+'RealOutputs/AngularControllers/21/PositionCommandResult', [])
print(json.dumps(result,indent=2))
