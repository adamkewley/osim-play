#!/usr/bin/env python3

import pandas
import matplotlib

df_simbody = pandas.read_csv("/tmp/simbody_version.csv")
df_opensim = pandas.read_csv("/tmp/opensim_version.csv")

fig, ax = matplotlib.pyplot.subplots()
df_simbody.plot(ax=ax, kind="line", x=0, y=1)
df_opensim.plot(ax=ax, kind="line", x=0, y=1)
ax.set_xlabel("simulation time")
ax.set_ylabel("lhs.getCoordinate().getValue() or lhs.getQ()")
ax.legend(["simbody /w cables", "OpenSim /w WrappingCylinder"])
ax.set_title("Q coordinate of left slider in basic wrapping simulation")
matplotlib.pyplot.show()
matplotlib.pyplot.close()
