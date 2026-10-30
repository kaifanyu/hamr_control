"""Evaluate existing predicted hemispheres only during observed inferred contact.

Read-only with respect to all_caster_reconstruction. No model re-integration,
phase fitting, phase reset at contact, or encoder feedback into prediction.
"""
from pathlib import Path
import argparse
from _paths import DATA_ROOT, OUTPUT_ROOT, reconstruction_source, prepared_inputs, relocated
import hashlib, json, math, shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ROOT=DATA_ROOT
SOURCE=reconstruction_source()
OUT=OUTPUT_ROOT/'caster_trajectory_replay/contact'
OUT.mkdir(parents=True,exist_ok=True)
R=.1;U_GAP=.1;U_POLE=math.sqrt(1-.25**2);TAU=2*np.pi
WHEELS=('front_left','front_right')
BOUNDARIES=np.sort(np.r_[np.arccos([-U_POLE,-U_GAP,U_GAP,U_POLE]),
                        TAU-np.arccos([-U_POLE,-U_GAP,U_GAP,U_POLE])])
COLORS={'observed':'#c67535','predicted':'#176c93','residual':'#a04770'}

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def sector(x):
    u=np.cos(x)
    return np.where(abs(u)<U_GAP,0,np.where(abs(u)>U_POLE,3,np.where(u<0,1,2)))

def fractions(delta):
    """Exact mode fractions for piecewise-linear observed CENTER in each step."""
    result=np.zeros((len(delta)-1,4))
    for i,(a,b) in enumerate(zip(delta[:-1],delta[1:])):
        if abs(b-a)<1e-14:result[i,int(sector((a+b)/2))]=1;continue
        lo,hi=sorted((a,b));f=[0.,1.]
        for k in range(math.floor(lo/TAU)-1,math.floor(hi/TAU)+2):
            for edge in BOUNDARIES+TAU*k:
                value=(edge-a)/(b-a)
                if 1e-12<value<1-1e-12:f.append(float(value))
        f=sorted(set(f))
        for x,y in zip(f[:-1],f[1:]):
            result[i,int(sector(a+(b-a)*(x+y)/2))]+=y-x
    return result

def stats(x):
    x=np.asarray(x,dtype=float);x=x[np.isfinite(x)]
    if not len(x):return dict(n=0,rmse=None,mean=None,median_abs=None,p95_abs=None,max_abs=None,last=None)
    return dict(n=len(x),rmse=float(np.sqrt(np.mean(x*x))),mean=float(x.mean()),
        median_abs=float(np.median(abs(x))),p95_abs=float(np.quantile(abs(x),.95)),
        max_abs=float(abs(x).max()),last=float(x[-1]))

def true_runs(mask):
    return zip(np.flatnonzero(mask & np.r_[True,~mask[:-1]]),
               np.flatnonzero(mask & np.r_[~mask[1:],True])+1)

def aggregate(parts):
    out={}
    for wheel in WHEELS:
        out[wheel]={}
        for h in [0,1,2]:
            q=[p[wheel][h] for p in parts]
            cat=lambda key:np.concatenate([x[key] for x in q])
            ns=sum(x['total_samples'] for x in q);n=sum(x['active_samples'] for x in q)
            total=sum(x['duration_s'] for x in q);contact=sum(x['contact_s'] for x in q)
            agreement=sum(x['agreement_samples'] for x in q)
            intervals=[v for x in q for v in x['episodes']]
            sampled=[v for x in q for v in x['sampled_episodes']]
            name='center' if h==0 else 'h'+str(h)
            out[wheel][name]={
                'observed_inferred_contact_samples':n,'total_samples':ns,
                'observed_inferred_contact_sample_fraction':n/ns,
                'observed_inferred_contact_duration_s':contact,
                'observed_inferred_contact_time_fraction':contact/total,
                'predicted_contact_agreement_fraction_of_observed_contact_samples':agreement/n if n else None,
                'predicted_agreement_basis':'Any predicted shell (H1 or H2), because CENTER belongs to the carrier.' if h==0 else 'Same predicted hemisphere as the observed inferred contact.',
                'primary_observed_contact_unwrapped_phase_error_turns':stats(cat('phase_error')),
                'primary_strict_observed_contact_rate_error_turns_s':stats(cat('rate_error')),
                'primary_strict_observed_contact_rate_duration_s':sum(x['rate_duration_s'] for x in q),
                'conditional_observed_and_predicted_contact_unwrapped_phase_error_turns':stats(cat('intersection_phase_error')),
                'conditional_strict_both_contact_rate_error_turns_s':stats(cat('intersection_rate_error')),
                'conditional_strict_both_contact_rate_duration_s':sum(x['intersection_rate_duration_s'] for x in q),
                'strict_observed_contact_episode_increment_error_turns':stats([x['error_turns'] for x in intervals]),
                'sampled_contact_episode_increment_error_turns':stats([x['error_turns'] for x in sampled]),
                'contact_episode_increment_origin':'First valid observed-contact sample in each episode; threshold crossing itself is not interpolated.',
                'contact_episodes':sampled,'strict_whole_step_contact_episodes':intervals}
            if h:out[wheel][name]['predicted_same_hemisphere_fraction_of_observed_contact_samples']=agreement/n if n else None
    return out

def analyze_segment(path):
    with np.load(path) as src:d={k:src[k].copy() for k in src.files}
    t=d['time_s'];dt=np.diff(t);et=d['case_elapsed_s'];measure={};plotdata={}
    for w in WHEELS:
        p=w+'__';delta=d[p+'observed_delta_rad'];obs_mode=sector(delta);fr=fractions(delta)
        assert np.allclose(fr.sum(axis=1),1,atol=1e-12)
        pred=d[p+'primary_change_rad']/TAU;obs=d[p+'observed_change_rad']/TAU
        pr=d[p+'primary_rate_mid_rad_s']/TAU;orr=d[p+'observed_rate_mid_rad_s']/TAU
        pm=d[p+'mode'];pf=d[p+'mode_fraction'];measure[w]={};plotdata[w]={}
        for h in [0,1,2]:
            mask=np.isin(obs_mode,[1,2]) if h==0 else obs_mode==h
            predmask=np.isin(pm,[1,2]) if h==0 else pm==h
            both=mask&predmask
            fraction=fr[:,1]+fr[:,2] if h==0 else fr[:,h]
            pred_fraction=pf[:,1]+pf[:,2] if h==0 else pf[:,h]
            strict=fraction>1-1e-10;strictboth=strict&(pred_fraction>1-1e-10)
            error=pred[:,h]-obs[:,h];rate_error=pr[:,h]-orr[:,h]
            episodes=[]
            for a,b in true_runs(strict):
                dp=pred[b,h]-pred[a,h];do=obs[b,h]-obs[a,h]
                episodes.append(dict(segment=path.stem,start_case_elapsed_s=float(et[a]),
                    end_case_elapsed_s=float(et[b]),duration_s=float(t[b]-t[a]),
                    predicted_increment_turns=float(dp),recorded_increment_turns=float(do),error_turns=float(dp-do)))
            sampled_episodes=[];increment_pred=np.full(len(t),np.nan);increment_obs=increment_pred.copy()
            for a,b in true_runs(mask):
                increment_pred[a:b]=pred[a:b,h]-pred[a,h]
                increment_obs[a:b]=obs[a:b,h]-obs[a,h]
                sampled_episodes.append(dict(segment=path.stem,start_case_elapsed_s=float(et[a]),
                    end_case_elapsed_s=float(et[b-1]),duration_s=float(t[b-1]-t[a]),samples=int(b-a),
                    entry_definition='First valid observed-contact sample; no threshold interpolation.',
                    predicted_increment_turns=float(increment_pred[b-1]),recorded_increment_turns=float(increment_obs[b-1]),
                    error_turns=float(increment_pred[b-1]-increment_obs[b-1])))
            measure[w][h]=dict(total_samples=len(t),active_samples=int(mask.sum()),duration_s=float(t[-1]-t[0]),
                contact_s=float((fraction*dt).sum()),agreement_samples=int(both.sum()),
                phase_error=error[mask],rate_error=rate_error[strict],
                rate_duration_s=float(dt[strict].sum()),
                intersection_phase_error=error[both],intersection_rate_error=rate_error[strictboth],
                intersection_rate_duration_s=float(dt[strictboth].sum()),episodes=episodes,sampled_episodes=sampled_episodes)
            plotdata[w][h]=dict(pred=np.where(mask,pred[:,h],np.nan),obs=np.where(mask,obs[:,h],np.nan),
                residual=np.where(mask,error,np.nan),
                pred_rate=np.where(strict,pr[:,h],np.nan),obs_rate=np.where(strict,orr[:,h],np.nan),
                rate_residual=np.where(strict,rate_error,np.nan),mask=mask)
            plotdata[w][h].update(increment_pred=increment_pred,increment_obs=increment_obs)
    return dict(path=path,elapsed=et,mid_elapsed=(et[:-1]+et[1:])/2,measure=measure,plot=plotdata)

def plot_case(case,parts,stats_case):
    tags={'phase':('Cumulative joint change (turns)','Caster joint rotation during inferred observed shell contact'),
          'residual':('Prediction − recorded (turns)','Unwrapped caster joint error during inferred observed shell contact'),
          'rate':('Joint rate (turns/s)','Caster joint rates during whole-step inferred observed shell contact'),
          'episode_increment':('Change since contact entry (turns)','Caster joint rotation within each inferred contact episode')}
    stamp=case.split('T')[1].split('Z')[0];clock=stamp[:2]+':'+stamp[2:4]+':'+stamp[4:]+' UTC'
    files={}
    for kind,(ylabel,title) in tags.items():
        fig,axs=plt.subplots(3,2,figsize=(13.4,10.9),sharex=True)
        fig.subplots_adjust(left=.083,right=.982,bottom=.105,top=.827,hspace=.39,wspace=.20)
        for col,w in enumerate(WHEELS):
            for row,h in enumerate([0,1,2]):
                name='center' if h==0 else 'h'+str(h);display='CENTER' if h==0 else f'H{h}'
                ax=axs[row,col];metrics=stats_case[w][name]
                for part in parts:
                    q=part['plot'][w][h]
                    if kind=='phase':
                        ax.plot(part['elapsed'],q['obs'],color=COLORS['observed'],lw=1.35)
                        ax.plot(part['elapsed'],q['pred'],color=COLORS['predicted'],lw=1.25)
                    elif kind=='residual':
                        ax.plot(part['elapsed'],q['residual'],color=COLORS['residual'],lw=1.3)
                    elif kind=='rate':
                        ax.plot(part['mid_elapsed'],q['obs_rate'],color=COLORS['observed'],lw=1.1)
                        ax.plot(part['mid_elapsed'],q['pred_rate'],color=COLORS['predicted'],lw=1.1)
                    else:
                        ax.plot(part['elapsed'],q['increment_obs'],color=COLORS['observed'],lw=1.25)
                        ax.plot(part['elapsed'],q['increment_pred'],color=COLORS['predicted'],lw=1.25)
                for k,part in enumerate(parts[1:],1):
                    a=parts[k-1]['elapsed'][-1];b=part['elapsed'][0]
                    ax.axvspan(a,b,color='#dadee4',alpha=.85,zorder=0)
                    ax.axvline(b,color='#626b76',ls=':',lw=.9)
                ax.axhline(0,color='#a8afb8',lw=.6,zorder=0)
                cov=100*metrics['observed_inferred_contact_sample_fraction']
                key='primary_strict_observed_contact_rate_error_turns_s' if kind=='rate' else 'primary_observed_contact_unwrapped_phase_error_turns'
                if kind=='episode_increment':key='sampled_contact_episode_increment_error_turns'
                rms=metrics[key]['rmse'];unit='turns/s' if kind=='rate' else 'turns'
                metricname='final-increment RMSE' if kind=='episode_increment' else 'RMSE'
                ax.set_title(f'{"Left" if col==0 else "Right"} {display}  ·  contact {cov:.1f}%  ·  {metricname} {rms:.3f} {unit}' if rms is not None else f'{w} {display}: no eligible samples',fontsize=10.1,loc='left',pad=8)
                ax.grid(True,color='#d9dee5',alpha=.5,lw=.5);ax.spines[['top','right']].set_visible(False)
                ax.set_xlim(0,max(float(p['elapsed'][-1]) for p in parts))
                if col==0:ax.set_ylabel(ylabel,fontsize=10)
                if row==2:ax.set_xlabel('Case elapsed time (s)',fontsize=10)
                ax.tick_params(labelsize=9)
        fig.suptitle(f'{clock}  |  {title}',fontsize=15,fontweight='bold',x=.083,ha='left',y=.965)
        fig.text(.083,.928,'Both curves use the same encoder-inferred contact mask. Gap and polar-opening sectors are excluded.',fontsize=10,color='#344351')
        fig.text(.083,.906,'CENTER uses either shell’s contact; H1/H2 use their own contact sectors. This is geometric inference, not a contact-sensor measurement.',fontsize=9.3,color='#566270')
        fig.text(.083,.884,'100 mm sphere; 20 mm equatorial gap; 50 mm polar opening.',fontsize=9.3,color='#566270')
        if kind!='residual':
            handles=[Line2D([0],[0],color=COLORS['observed'],lw=2,label='Recorded encoder'),
                     Line2D([0],[0],color=COLORS['predicted'],lw=2,label='Existing ideal prediction')]
            fig.legend(handles=handles,loc='upper left',bbox_to_anchor=(.078,.882),frameon=False,ncol=2,fontsize=10)
        note='Axial phases keep the original valid-segment origin; they are never reset at individual contact episodes.'
        if kind=='episode_increment':note='Change since inferred contact entry; zero at entry by definition. Entry = first valid contact sample (no threshold interpolation).'
        if len(parts)>1:note+=' Gray gap / dotted line: data exclusion and fresh segment initialization.'
        fig.text(.083,.043,note,fontsize=9,color='#566270')
        if kind=='rate':fig.text(.083,.022,'Rates use 10 ms finite differences only when the entire observed interval lies within the selected shell-contact sector(s).',fontsize=9,color='#566270')
        elif kind=='episode_increment':fig.text(.083,.022,'Each curve subtracts its own value at that episode’s entry for this diagnostic only. The stored prediction and cumulative comparison are unchanged.',fontsize=9,color='#566270')
        else:fig.text(.083,.022,'Unwrapped turns preserve complete revolutions. Empty portions are intentionally hidden; lines do not bridge inactive contact intervals.',fontsize=9,color='#566270')
        path=OUT/(case+f'__contact_{kind}.png');fig.savefig(path,dpi=160,facecolor='white');plt.close(fig)
        files[kind]=path.name
    return files

def checks():
    assert np.array_equal(sector(np.array([0,np.pi/2,np.pi,3*np.pi/4,np.pi/4])),[3,0,3,1,2])
    # Equal endpoint sectors must not make a full revolution a whole-contact step.
    full=fractions(np.array([np.pi/4,np.pi/4+TAU]))[0]
    assert np.all(full>0) and full[2]<1
    stationary=fractions(np.array([np.pi/4,np.pi/4]))[0]
    assert np.array_equal(stationary,[0,0,1,0])
    forward=fractions(np.array([-.2,3.9]))[0];reverse=fractions(np.array([3.9,-.2]))[0]
    assert np.allclose(forward,reverse,atol=1e-12)
    return {'all_four_sector_codes_checked':True,'full_turn_endpoint_alias_rejected':True,
            'stationary_shell_is_full_contact':True,'contact_fraction_path_reversal_invariant':True}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',help='Exact recording folder name; omitted means all.')
    args=parser.parse_args()
    groups={}
    for path in sorted((SOURCE/'histories').glob('*.npz')):
        case=path.stem.split('__segment')[0]
        if args.case and case != args.case:continue
        groups.setdefault(case,[]).append(path)
    if not groups:raise ValueError('No reconstructed histories matched the requested case(s).')
    summary=dict(schema_version=1,source_directory=str(SOURCE/'histories'),source_results_sha256=sha(SOURCE/'results.json'),
        script_sha256=sha(__file__),geometry={'radius_m':R,'equatorial_gap_m':.02,'polar_opening_diameter_m':.05},
        primary_mask='Observed encoder CENTER: 0.1 <= abs(cos(delta_observed)) <= sqrt(1-0.25^2); H1 cos<0, H2 cos>0.',
        center_mask='Union of observed inferred H1/H2 shell eligibility; CENTER is evaluated while either hemisphere is eligible. Predicted conditional agreement also uses the H1/H2 union.',
        primary_evaluation='Apply the same observed inferred contact mask to existing prediction and recorded phase. Prediction is not recomputed or conditioned on future encoders.',
        conditional_evaluation='Separately reported intersection with the same predicted hemisphere; diagnostic only, not primary selection.',
        rate_evaluation='10 ms finite differences; only entire observed steps within the hemisphere sector. Exact threshold splitting of piecewise-linear observed CENTER rejects crossing steps.',
        phase_definition='Original cumulative axial change from each valid segment initial reading; no reset at individual contact episodes. No angle wrapping.',
        exclusions='Inferred equatorial gap and polar opening are excluded. This geometric eligibility is not a measured contact sensor or a solved rim/roller support model.',
        assumptions=['No source history modified. No new clock, radius, yaw, phase, sign, scale or delay fit.',
            'Predictions retain the prior hold-inactive axial-spin closure.',
            'H1/H2 encoder scale and sign remain the original unverified convention.',
            'Aggregates pool sample errors across separately initialized valid segments; no phase continuity is invented across data gaps.',
            'Contact episode plots subtract each curve’s own first valid observed-contact sample. This is a diagnostic difference, not a model reset or phase correction.',
            'Sampled contact-episode final increments and strict whole-contact step increments are separately recorded; primary cumulative errors keep the original segment phase origin.'],
        analytical_checks=checks(),cases=[],segments=[])
    all_parts=[]
    # Emit the selected case first so its metrics are available during rendering.
    order=sorted(groups,key=lambda c:('005046' not in c,c))
    for case in order:
        parts=[analyze_segment(path) for path in groups[case]]
        measures=[p['measure'] for p in parts];metrics=aggregate(measures);all_parts.extend(measures)
        caseitem=dict(case=case,segments=[p['path'].stem for p in parts],
            files=plot_case(case,parts,metrics),wheels=metrics,
            available_duration_s=sum(float(p['elapsed'][-1]-p['elapsed'][0]) for p in parts))
        summary['cases'].append(caseitem)
        for p in parts:
            summary['segments'].append(dict(segment=p['path'].stem,source_sha256=sha(p['path']),
                case_elapsed_interval_s=[float(p['elapsed'][0]),float(p['elapsed'][-1])],wheels=aggregate([p['measure']])))
        (OUT/(case+'__contact_metrics.json')).write_text(json.dumps(caseitem,indent=2))
        print(json.dumps({'case':case,'wheels':{w:{h:{'contact_fraction':v['observed_inferred_contact_sample_fraction'],
            'predicted_agreement':v['predicted_contact_agreement_fraction_of_observed_contact_samples'],
            'phase_rmse_turns':v['primary_observed_contact_unwrapped_phase_error_turns']['rmse'],
            'rate_rmse_turns_s':v['primary_strict_observed_contact_rate_error_turns_s']['rmse']} for h,v in metrics[w].items()} for w in WHEELS}}),flush=True)
    summary['cases'].sort(key=lambda c:c['case']);summary['case_count']=len(groups)
    summary['segment_count']=len(summary['segments']);summary['pooled_all_cases']=aggregate(all_parts)
    (OUT/'contact_metrics.json').write_text(json.dumps(summary,indent=2))
    if Path(__file__).resolve() != (OUT/'contact_analysis.py').resolve():
        shutil.copy2(__file__,OUT/'contact_analysis.py')
    print('SUMMARY '+str(OUT/'contact_metrics.json'),flush=True)

if __name__=='__main__':main()
