import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from fk_irb120 import forward_kinematics, DH_TABLE

NAR="#d2601a"; GRIS="#5a6672"; OSC="#1f2a33"
plt.rcParams.update({"font.size":9,"font.family":"serif","axes.edgecolor":GRIS})

# ---- Fig 1: origenes en q=0, plano x-z
q0=np.zeros(6)
pts=[np.zeros(3)]+[forward_kinematics(q0,upto=i)[:3,3] for i in range(1,7)]
P=np.array(pts)
fig,ax=plt.subplots(figsize=(5.4,4.2))
ax.plot(P[:,0],P[:,2],"-o",color=NAR,lw=2.2,ms=6,zorder=3)

etq=[(0,"$O_0$",(12,-4)),(1,"$O_1$",(12,-4)),(2,"$O_2$",(12,-4)),
     (3,"$O_3$",(-6,12)),(6,"$O_6$",(10,6))]
for i,t,off in etq:
    ax.annotate(t,(P[i,0],P[i,2]),textcoords="offset points",
                xytext=off,color=OSC,fontsize=9)
ax.annotate("$O_4=O_5$",(P[4,0],P[4,2]),textcoords="offset points",
            xytext=(-6,-20),color=OSC,fontsize=9,ha="center")

cot=[(0,1,"$d_1=0{,}290$"),(1,2,"$a_2=0{,}270$"),(2,3,"$a_3=0{,}070$")]
for a_,b_,txt in cot:
    mz=(P[a_,2]+P[b_,2])/2
    ax.text(-0.035,mz,txt,color=GRIS,fontsize=8,ha="right",va="center")
ax.text((P[3,0]+P[4,0])/2,P[3,2]+0.035,"$d_4=0{,}302$",color=GRIS,
        fontsize=8,ha="center")
ax.text((P[5,0]+P[6,0])/2,P[6,2]+0.055,"$d_6=0{,}072$",color=GRIS,
        fontsize=8,ha="center")
ax.annotate("centro de muñeca",(P[4,0],P[4,2]),textcoords="offset points",
            xytext=(4,-46),fontsize=8,color=GRIS,ha="center",
            arrowprops=dict(arrowstyle="->",color=GRIS,lw=0.8))

ax.set_xlabel("$x$ [m]"); ax.set_ylabel("$z$ [m]")
ax.set_xlim(-0.30,0.55); ax.set_ylim(-0.10,0.80)
ax.grid(alpha=0.25,ls=":"); ax.set_aspect("equal")
for s_ in ["top","right"]: ax.spines[s_].set_visible(False)
fig.tight_layout(); fig.savefig("img/origenes.pdf")

# ---- Fig 2: escalas de error
fig,ax=plt.subplots(figsize=(5.6,2.5))
lab=["Verificación\ninterna\n(float64)","Medición\nvía TF\n(float32)","Umbral\ndel hito"]
val=[3.68e-16,2.459e-10,1e-6]
col=[GRIS,NAR,OSC]
b=ax.barh(lab,val,color=col,height=0.55)
ax.set_xscale("log"); ax.set_xlim(1e-17,1e-4)
ax.set_xlabel("Error posicional [m], escala logarítmica")
for r,v in zip(b,val):
    ax.text(v*1.6,r.get_y()+r.get_height()/2,f"{v:.2e}",va="center",fontsize=8,color=OSC)
ax.grid(axis="x",alpha=0.25,ls=":")
for s in ["top","right","left"]: ax.spines[s].set_visible(False)
fig.tight_layout(); fig.savefig("img/errores.pdf")

# ---- Fig 3: convergencia del peor error
n=[300,400,500,600,700,800,900,1000]
e=[2.457e-10]*5+[2.459e-10]*3
fig,ax=plt.subplots(figsize=(5.6,2.4))
ax.plot(n,e,"-o",color=NAR,lw=2,ms=5)
ax.axhline(1e-6,color=OSC,ls="--",lw=1)
ax.text(320,1.2e-6,"umbral del hito $10^{-6}$ m",fontsize=8,color=OSC)
ax.set_yscale("log"); ax.set_ylim(1e-10,1e-5)
ax.set_xlabel("configuraciones publicadas"); ax.set_ylabel("peor error [m]")
ax.grid(alpha=0.25,ls=":")
for s in ["top","right"]: ax.spines[s].set_visible(False)
fig.tight_layout(); fig.savefig("img/convergencia.pdf")

# ---- Fig 4: pruebas de juntas individuales
fig,ax=plt.subplots(figsize=(5.6,2.6))
js=[];ds=[]
for i in range(6):
    q=np.zeros(6); q[i]=0.5
    p=forward_kinematics(q)[:3,3]
    js.append(f"J{i+1}"); ds.append(np.linalg.norm(p-np.array([0.374,0,0.630])))
cols=[NAR if d>1e-9 else GRIS for d in ds]
b=ax.bar(js,ds,color=cols,width=0.55)
for r,d in zip(b,ds):
    ax.text(r.get_x()+r.get_width()/2,d+0.006,f"{d:.4f}",ha="center",fontsize=8,color=OSC)
ax.set_ylabel("desplazamiento de tool0 [m]")
ax.set_ylim(0,0.21)
ax.text(4.15,0.155,"J4 y J6 no desplazan la brida:\nsu eje es colineal con el vector hacia tool0",
        fontsize=8,color=GRIS,ha="center")
ax.grid(axis="y",alpha=0.25,ls=":")
for s in ["top","right"]: ax.spines[s].set_visible(False)
fig.tight_layout(); fig.savefig("img/juntas.pdf")
print("figuras ok")
