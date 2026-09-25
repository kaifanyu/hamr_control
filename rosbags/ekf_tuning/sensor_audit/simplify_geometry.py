#!/usr/bin/env python3
"""Cross-validate geometry while keeping wheel asymmetry exactly neutral."""
import json
import numpy as np
from scipy.optimize import least_squares
from fit_geometry import OUT, load, predict, metrics

bags=load();results={}
def fit(train,delay,mode='wheel'):
    def expand(q):return [q[0],0,q[1],q[2]]+([q[3]] if delay else [])
    def residual(q):
        p=expand(q)
        return np.concatenate([((predict(b,p,mode)-b['truth'])[b['mask']][::5]*[1,1,1])/np.sqrt(sum(b['mask'])/5) for b in train]).ravel()
    r=least_squares(residual,[.96,.82,.28]+([.15] if delay else []),bounds=([.8,.7,.1]+([0] if delay else []),[1.2,1,.5]+([.35] if delay else [])),x_scale='jac',max_nfev=250)
    return expand(r.x)
for mode in ('wheel','blend'):
 for delay in (False,True):
    p=fit(bags,delay,mode);label=mode+('_delay' if delay else '')
    results[label]={'parameters':p,'scores':{b['name']:metrics(b,p,mode) for b in bags},'heldout':[]}
    for i,b in enumerate(bags):
        pp=fit([bb for j,bb in enumerate(bags) if i!=j],delay,mode)
        results[label]['heldout'].append(dict(bag=b['name'],parameters=pp,score=metrics(b,pp,mode)))
for p in ([.955,0,.821,.26],[.96,0,.82,.28],[.96,0,.82,.28,.15],[.96,0,.82,.301],[.96,0,.82,.301,.15]):
    results[str(p)]={b['name']:metrics(b,p) for b in bags}
(OUT/'simplified_geometry.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
