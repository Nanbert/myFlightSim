// Minimal C-ABI shim over JSBSim's FGFDMExec. This is the only translation
// unit that touches C++; everything above it speaks plain C types.

#include <FGFDMExec.h>
#include <FGInitialCondition.h>
#include <sg_path.hxx>

#include <string>

using namespace JSBSim;

extern "C" {

void* jsbsim_new() {
    return new FGFDMExec();
}

void jsbsim_free(void* fdm) {
    delete static_cast<FGFDMExec*>(fdm);
}

void jsbsim_set_root(void* fdm, const char* root) {
    static_cast<FGFDMExec*>(fdm)->SetRootDir(SGPath(std::string(root)));
}

void jsbsim_set_aircraft_path(void* fdm, const char* path) {
    static_cast<FGFDMExec*>(fdm)->SetAircraftPath(SGPath(std::string(path)));
}

void jsbsim_set_engine_path(void* fdm, const char* path) {
    static_cast<FGFDMExec*>(fdm)->SetEnginePath(SGPath(std::string(path)));
}

void jsbsim_set_systems_path(void* fdm, const char* path) {
    static_cast<FGFDMExec*>(fdm)->SetSystemsPath(SGPath(std::string(path)));
}

void jsbsim_set_dt(void* fdm, double dt) {
    static_cast<FGFDMExec*>(fdm)->Setdt(dt);
}

int jsbsim_load_model(void* fdm, const char* name) {
    return static_cast<FGFDMExec*>(fdm)->LoadModel(std::string(name)) ? 1 : 0;
}

int jsbsim_run_ic(void* fdm) {
    return static_cast<FGFDMExec*>(fdm)->RunIC() ? 1 : 0;
}

int jsbsim_run(void* fdm) {
    return static_cast<FGFDMExec*>(fdm)->Run() ? 1 : 0;
}

void jsbsim_set_property(void* fdm, const char* name, double value) {
    static_cast<FGFDMExec*>(fdm)->SetPropertyValue(std::string(name), value);
}

double jsbsim_get_property(void* fdm, const char* name) {
    return static_cast<FGFDMExec*>(fdm)->GetPropertyValue(std::string(name));
}

void jsbsim_ic_speed_kts(void* fdm, double kts) {
    static_cast<FGFDMExec*>(fdm)->GetIC()->SetVcalibratedKtsIC(kts);
}

void jsbsim_ic_altitude_ft(void* fdm, double ft) {
    static_cast<FGFDMExec*>(fdm)->GetIC()->SetAltitudeASLFtIC(ft);
}

void jsbsim_ic_heading_deg(void* fdm, double deg) {
    static_cast<FGFDMExec*>(fdm)->GetIC()->SetPsiDegIC(deg);
}

} // extern "C"
