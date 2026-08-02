#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

int main(int argc,char** argv) {
  if (argc!=2) { std::fprintf(stderr,"用法: %s model.xml\n",argv[0]); return 1; }
  char error[1024]={0}; mjModel* m=mj_loadXML(argv[1],NULL,error,sizeof(error));
  if (!m) { std::fprintf(stderr,"%s\n",error); return 1; }
  mjData* d=mj_makeData(m); int pelvis=mj_name2id(m,mjOBJ_BODY,"pelvis");
  int left=mj_name2id(m,mjOBJ_GEOM,"left_foot_geom");
  int right=mj_name2id(m,mjOBJ_GEOM,"right_foot_geom");
  for (int k=0;k<2000;++k) { for (int i=0;i<m->nu;++i)d->ctrl[i]=0; mj_step(m,d); }
  double fz[2]={0,0}; int contacts[2]={0,0};
  for (int i=0;i<d->ncon;++i) {
    const mjContact& c=d->contact[i];
    int which=(c.geom[0]==left||c.geom[1]==left)?0:(c.geom[0]==right||c.geom[1]==right)?1:-1;
    if (which<0) continue;
    mjtNum w[6]; mj_contactForce(m,d,i,w);
    double world_z=c.frame[2]*w[0]+c.frame[5]*w[1]+c.frame[8]*w[2];
    fz[which]+=world_z; ++contacts[which];
  }
  double weight=m->body_subtreemass[0]*std::fabs(m->opt.gravity[2]);
  double support_error=std::fabs(fz[0]+fz[1]-weight)/weight;
  const mjtNum* com=d->subtree_com+3*pelvis;
  bool pass=support_error<.05 && d->xpos[3*pelvis+2]>.65 && contacts[0]&&contacts[1];
  std::printf("nq=%lld nv=%lld nu=%lld mass=%.3f kg\n",
              (long long)m->nq,(long long)m->nv,(long long)m->nu,m->body_subtreemass[0]);
  std::printf("base_z=%.6f m, whole-body CoM=[%.4f %.4f %.4f]\n",
              d->xpos[3*pelvis+2],com[0],com[1],com[2]);
  std::printf("left Fz=%.3f N (%d contacts), right Fz=%.3f N (%d contacts)\n",
              fz[0],contacts[0],fz[1],contacts[1]);
  std::printf("weight=%.3f N, relative support error=%.3f%%, %s\n",
              weight,100*support_error,pass?"PASS":"FAIL");
  mj_deleteData(d); mj_deleteModel(m); return pass?0:2;
}
