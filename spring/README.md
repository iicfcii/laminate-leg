## Test geometry

**V1:**

![test setup geometry](https://raw.githubusercontent.com/iicfcii/laminate-leg/rotational-spring/spring/characterization-v1/test-geometry.png)

**V2:**

![test setup geometry](https://raw.githubusercontent.com/iicfcii/laminate-leg/rotational-spring/spring/characterization-v2/test-geometry.png)

## Characterization

1. Open desired version of `characterize-spring.py` and set the correct values for the experiment parameters and structure geometry.
2. Update `tz.csv` with the desire torque data. Put multiple trials in separate columns.
3. Run `characterize-spring.py`.  Figures and data files will be generated for each column in `tz.csv`.