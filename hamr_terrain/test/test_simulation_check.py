import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

MODULE = Path(__file__).resolve().parents[1]/"scripts/terrain_simulation_check.py"
spec=importlib.util.spec_from_file_location("terrain_simulation_check",MODULE)
check=importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


def grid(function=lambda x,y:0*x):
    x=np.linspace(-4,4,321); y=np.linspace(-4,4,321)
    xx,yy=np.meshgrid(x,y)
    return {"x":x,"y":y,"z":function(xx,yy)}


def test_sphere_support_on_slope_is_not_point_height_plus_radius():
    g=grid(lambda x,y:.3*x+.15*y)
    expected=.3*.4+.15*.2+.04*np.sqrt(1+.3**2+.15**2)
    assert check.support_height(g,[.4,.2],.04)==pytest.approx(expected,abs=2e-6)


def test_sphere_support_respects_triangle_diagonal():
    # a=c=1, b=d=0: the a-c diagonal is a ridge, not a bilinear saddle.
    g={"x":np.array([0.,1.]),"y":np.array([0.,1.]),"z":np.array([[1.,0.],[0.,1.]])}
    assert check.support_height(g,[.5,.5],0.)==pytest.approx(1.)
    assert check.support_height(g,[.5,.5],.04)==pytest.approx(1.04)


def test_drop_fixture_is_passive_vertical_and_has_contact_sensor():
    p=check.probe_specs({"probes":[{"name":"feature","xy_m":[0,0]}]},grid())[0]
    model=ET.fromstring(check.probe_sdf(p)).find("model")
    assert model.find("joint").get("type")=="prismatic"
    assert model.findtext("joint/parent")=="world"
    assert model.findtext("joint/axis/xyz")=="0 0 1"
    assert model.find("link/sensor").get("type")=="contact"
    assert len(model.findall("plugin"))==1
    assert "OdometryPublisher" in model.find("plugin").get("name")


def test_sliding_probe_uses_box_with_requested_friction_and_no_guide():
    probes=check.probe_specs({"probes":[]},grid())
    boxes=[p for p in probes if p["kind"]=="sliding_box"]
    assert [p["expected_combined_mu"] for p in boxes]==[.3,1.]
    for p in boxes:
        model=ET.fromstring(check.probe_sdf(p)).find("model")
        assert model.find("joint") is None
        assert model.findtext("link/collision/geometry/box/size")==".06 .06 .04"
        assert float(model.findtext("link/collision/surface/friction/ode/mu"))==p["friction"]
        assert float(model.findtext("link/inertial/inertia/izz"))==pytest.approx(.2*(.06**2+.06**2)/12)


def test_measured_settling_requires_contacts_and_correct_height():
    spec={"name":"drop","kind":"guided_drop","support_z_m":.04,"xy_m":[0.,0.]}
    samples=[{"stamp_s":float(t),"position_m":[0,0,.26-.22*min(1,t)],"linear_velocity_m_s":[0,0,0]} for t in np.linspace(0,3,151)]
    contacts=[{"depths_m":[.0002],"collision_names":["terrain::surface::collision","drop::body::shape"]}]
    report=check.summarize_probe(spec,samples,contacts,None)
    assert report["passed"]
    assert not check.summarize_probe(spec,samples,[],None)["passed"]
    raised=[dict(s,position_m=[0,0,s["position_m"][2]+.03]) for s in samples]
    assert not check.summarize_probe(spec,raised,contacts,None)["checks"]["support_height"]


def test_sliding_mu_uses_post_pulse_deceleration():
    spec={"kind":"sliding_box","expected_combined_mu":.3}
    samples=[{"stamp_s":float(t),"position_m":[float(t),0,.02],
              "linear_velocity_m_s":[max(0.,1.-.3*9.80665*(t-1.)),0,0]} for t in np.linspace(0,2,201)]
    contacts=[{"depths_m":[.0001],"collision_names":["terrain::collision"]}]
    report=check.summarize_probe(spec,samples,contacts,{"start_s":.8,"stop_s":1.})
    assert report["passed"]
    assert report["measured_sliding_mu"]==pytest.approx(.3)
    spec["expected_combined_mu"]=1.
    assert not check.summarize_probe(spec,samples,contacts,{"start_s":.8,"stop_s":1.})["passed"]


def test_snapshot_preserves_collision_and_physics_except_rtf(tmp_path):
    source=tmp_path/"source.sdf"
    source.write_text('''<sdf version="1.9"><world name="terrain"><physics name="p" type="ignored"><max_step_size>.001</max_step_size><real_time_factor>1</real_time_factor></physics><model name="terrain"><static>true</static><link name="body"><collision name="terrain_collision"><geometry><mesh><uri>meshes/test.stl</uri></mesh></geometry><surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision></link></model></world></sdf>''')
    output=tmp_path/"snapshot.sdf"
    assert check.prepare_world(source,output,.4)=="terrain"
    world=ET.parse(output).getroot().find("world")
    assert world.findtext("physics/max_step_size")==".001"
    assert world.findtext("physics/real_time_factor")=="0.4"
    assert world.findtext(".//collision/surface/friction/ode/mu")=="1"
    assert world.findtext(".//mesh/uri")==str(tmp_path/"meshes/test.stl")
    assert len(world.findall("plugin"))==2


def test_robot_smoke_requires_measured_rear_and_front_contact():
    samples=[{"position_m":[0,float(t)*.12,.01],"rpy_rad":[0,0,np.pi/2]} for t in np.linspace(0,10,51)]
    contacts=[{"depths_m":[.001],"collision_names":["terrain::surface::terrain_collision", "compa::left_wheel_link::collision", "compa::right_caster_wheel_link::collision"]}]
    assert check.robot_summary(samples,1.2,contacts)["passed"]
    assert not check.robot_summary(samples,1.2,[])["passed"]
    contacts[0]["collision_names"].pop()
    assert not check.robot_summary(samples,1.2,contacts)["checks"]["front_caster_contact_observed"]


def test_robot_smoke_rejects_fall_even_with_contacts():
    samples=[{"position_m":[0,float(t)*.12,-.2],"rpy_rad":[0,0,np.pi/2]} for t in np.linspace(0,10,51)]
    contacts=[{"depths_m":[.001],"collision_names":["terrain::surface::terrain_collision", "compa::left_wheel_link::collision", "compa::right_caster_wheel_link::collision"]}]
    assert not check.robot_summary(samples,1.2,contacts)["checks"]["above_terrain_bottom"]


def test_front_wheel_cannot_satisfy_rear_wheel_contact_check():
    samples=[{"position_m":[0,float(t)*.12,.01],"rpy_rad":[0,0,np.pi/2]} for t in np.linspace(0,10,51)]
    contacts=[{"depths_m":[.001],"collision_names":["terrain::surface::terrain_collision", "compa::left_caster_wheel_link::collision"]}]
    report=check.robot_summary(samples,1.2,contacts)
    assert not report["checks"]["rear_wheel_contact_observed"]
    assert report["checks"]["front_caster_contact_observed"]
    contacts[0]["collision_names"]=["terrain::surface::terrain_collision", "compa::left_wheel_link::collision", "compa::right_caster_fork_link::collision"]
    report=check.robot_summary(samples,1.2,contacts)
    assert not report["checks"]["front_caster_contact_observed"]
    assert report["other_caster_contact_names"]==["compa::right_caster_fork_link::collision"]


def test_sliding_speed_fit_does_not_confuse_box_yaw_with_deceleration():
    spec={"kind":"sliding_box","expected_combined_mu":.3}
    samples=[]
    for t in np.linspace(0,2,201):
        speed=max(0.,1.-.3*9.80665*(t-1.))
        yaw=1.2*t
        samples.append({"stamp_s":float(t),"position_m":[float(t),0,.02],
                        "linear_velocity_m_s":[speed*np.cos(yaw),-speed*np.sin(yaw),0]})
    contacts=[{"depths_m":[.0001],"collision_names":["terrain::collision"]}]
    result=check.summarize_probe(spec,samples,contacts,{"start_s":.8,"stop_s":1.})
    assert result["measured_sliding_mu"]==pytest.approx(.3)
    assert result["passed"]


def test_report_rejects_persistent_acceleration_after_requested_force_clear():
    spec={"name":"free_rolling","kind":"free_sphere","support_z_m":.04}
    samples=[{"stamp_s":float(t),"position_m":[.4*t*t,0,.04],
              "linear_velocity_m_s":[.8*t,0,0]} for t in np.linspace(0,2,201)]
    contacts=[{"depths_m":[.0001],"collision_names":["terrain::collision","free_rolling::body::shape"]}]
    report=check.summarize_probe(spec,samples,contacts,{"start_s":.8,"stop_s":1.})
    assert report["checks"]["free_motion"]
    assert report["post_clear_coast_acceleration_m_s2"]==pytest.approx(.8)
    assert not report["checks"]["force_off_observed"]
    assert not report["passed"]
