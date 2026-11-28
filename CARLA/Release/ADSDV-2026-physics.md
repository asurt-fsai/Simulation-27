# ADS-DV 2026 physics candidate

This update uses the 2026 reference vehicle as a starting point for CARLA. Measurements from the team car and the range of validity of the CARLA model matter separately when tuning a dynamic MPC.

## Sources and configuration

| Quantity | Source and treatment |
| --- | --- |
| Wheel geometry | The IMechE vehicle sheet specifies a 1.530 m wheelbase and 1.200 m front and rear tracks. The chassis authoring script uses those dimensions and sets the two wheel centres on each axle to the same height. |
| Operational vehicle mass | The IMechE HiL model lists a 279 kg body plus four 5.3 kg wheel carriers and four 6.2 kg wheels. The aggregate model mass is 325 kg. The assembled team-car mass is still needed. |
| CG and inertia | The same nine HiL bodies give a CG 0.76474 m behind the front axle and 0.76526 m ahead of the rear axle, 0.16472 m above the model ground datum. The parallel-axis theorem gives 94.6578 kg·m² for yaw. An in-engine yaw impulse is used to calibrate the CARLA asset's yaw inertia scale. The reference CG is transferred as an offset from the measured CARLA asset COM. |
| Suspension and load transfer | The wheel classes apply tire forces at the nominal contact plane and use mirrored hub heights. The authoring script converts the HiL spring rates to wheel rates using each axle's motion ratio near zero travel. It linearizes damping using the first compression and rebound points and uses the buffer engagement distances as travel limits. PhysX uses six vehicle substeps on either side of its speed threshold, which gives 5.5 ms steps at the packaged frame interval. The HiL suspension's nonlinear geometry and buffer response still require separate validation. |
| Differential and powertrain | The Unreal vehicle has an open rear differential. The supplied steering and traction configuration uses the native drivetrain and preserves grip on both axles. The Saietta 119R-68 is modeled with a 55.7 N·m peak, 4,000 RPM ceiling, and a single 3.5:1 reduction. The eight-point torque envelope uses the competition's published ratings and peaks at 4.89 kW of ideal interpolated shaft power. Full-throttle motor damping and supplemental wheel damping are reduced to avoid the sustained 24 km/h plateau produced by the legacy PhysX losses. The measured Sevcon Gen4 DC map, efficiency, coast losses, and regen calibration are unavailable. |
| Battery | The vehicle sheet identifies a four-module lead-acid pack, 100 Ah, nominal 50 V, operating at 48–52 V, and 100 A continuous / 200 A peak discharge. CARLA uses the power ceiling. Pack sag, current transients, and state-dependent torque limits lack vehicle calibration data. |
| Tires | The published 225/45R13 84V Toyo R888R listing gives 616 rev/km. That converts to a 0.258368 m effective rolling radius for the simulator. Nominal outside diameter and measured radius at the team's load and pressure are separate quantities. The HiL reference tire is a generated 185/50R13 model, so its stiffness and rolling resistance are unsuitable as Toyo calibration. |
| Aerodynamics | The IMechE HiL model uses Cd 0.467 and frontal area 0.49 m², with zero vertical aerodynamic coefficient at zero yaw. CARLA uses those drag inputs, equivalent to CdA 0.22883 m². |
| Maximum steering and speed | The current 2026 DBC specifies a ±30° virtual bicycle request. The CARLA steering scale is 30° at the inside front wheel, with the outer wheel following the chassis's Ackermann geometry. These limits use different angle definitions. The public HiL rack table reaches about ±27.61° virtual at ±48 mm. The physical linkage limits still need a team measurement. The keyboard reaches the full signed range in 1.5 s. The manual accelerator cutoff and both ROS speed target paths use the requested 30 m/s ceiling. |
| Steering feedback | The VCU CAN angle field and steering request use 0.1° per count. The distinct diagnostic actual and target fields use 0.5° per count. ROS per-wheel JointState positions now use radians with the ROS sign convention. The scalar steering feedback stays in degrees and reports the front-wheel average. |
| Actuator and hydraulic dynamics | The vehicle identifies a DC Electronics Microsteer and Shiftec ZE102-1. The documents do not give an ADS-DV steering actuator rate map, lag, or pressure-to-deceleration calibration. The keyboard request slope is 20°/s with the new range. CARLA measurements characterize the simulation and cannot establish those hardware parameters. |
| Alignment and tire forces | The specification lists 6° caster and 2° static camber without a camber sign. The tire multiplier gives a provisional dry asphalt coefficient of 1.0 on FlatAsphalt and Silverstone, whose measured dry road factor is 0.70. At reference tire temperature, a full water film reduces asphalt grip to 0.72 and maximum standing water reduces it to 0.65. Instrumented R888R force-versus-slip measurements are needed to validate cornering stiffness, load sensitivity, and combined slip. |

Source material

- [IMechE 2026 dimensions and component specs](https://github.com/FS-AI/FS-AI_ADS-DV_Documentation/blob/bb8f162/FS-AI%20ADS-DV%20Specifications%20%26%20Dimensions.md)
- [IMechE 2026 CAN database](https://github.com/FS-AI/FS-AI_ADS-DV_Documentation/blob/bb8f162/ADS-DV_2026_CAN1.dbc)
- [IMechE 2026 HiL car file](https://github.com/FS-AI/FS-AI_IMechE_ADS-DV_HiL/blob/1fe77bed411ba621a3187bc2eb9ca13c7d371832/Data/Vehicle/FS-AI_IMechE_ADS-DV_2026_HiL)
- [IMechE public CAD reference](https://github.com/FS-AI/FS-AI_ADS-DV_CAD/tree/958fdbf336fcc6cbd60535c2e7812d51a977d267)
- [IMechE HiL front kinematics](https://github.com/FS-AI/FS-AI_IMechE_ADS-DV_HiL/blob/1fe77bed411ba621a3187bc2eb9ca13c7d371832/Data/Chassis/FS_Autonomous_front.skc)
- [IMechE HiL rear kinematics](https://github.com/FS-AI/FS-AI_IMechE_ADS-DV_HiL/blob/1fe77bed411ba621a3187bc2eb9ca13c7d371832/Data/Chassis/FS_Autonomous_rear.skc)
- [Toyo R888R dimensions](https://www.toyotires.ca/fr/pneus/proxes-r888r)
- [Saietta 119R-68 component datasheet](https://faktor.de/out/media/BR-200-119-R-68.pdf)

## Build and verify

1. Audit a loaded FSAI asset's wheel-bone locations, COM, body mass, and inertia. Run the Unreal authoring script with the audit and the published HiL car file in its original `Data/Vehicle` directory, with the axle files in `Data/Chassis`. The optional yaw measurement input is the previous asset scale and its measured effective yaw inertia.
2. Cook the changed chassis and both wheel class assets for WindowsNoEditor and LinuxNoEditor. Repack the three cooked assets into the verified baseline runtimes. Keep the original shipping executables and validate all map pack dependencies.
3. Build the stack patch with the matching rebuilt ROS bridge. Supply the bridge executable to the channel packager. Its startup hash is recorded with the packaged physics config and the intended simulator cook.
4. Run the keyboard timing check, real wheel-angle and ROS feedback sweeps, yaw impulse and longitudinal tests using the new build. Check neutral steering on a level straight and mirror the corner-entry inputs on dry and flooded asphalt on both maps. Confirm the map and platform on each test. Keep maximum achievable speed and stopping distance tied to the tested conditions.
5. Publish to Testing and verify its runtime and stack hashes. Promote only after review.
