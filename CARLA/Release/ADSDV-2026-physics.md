# ADS-DV 2026 physics candidate

This update uses the 2026 reference vehicle as a starting point for CARLA. Measurements from the team car and the range of validity of the CARLA model matter separately when tuning a dynamic MPC.

## Sources and configuration

| Quantity | Source and treatment |
| --- | --- |
| Wheel geometry | The IMechE vehicle sheet specifies a 1.530 m wheelbase and 1.200 m front and rear tracks. The chassis authoring script uses those dimensions as wheel-centre targets. |
| Operational vehicle mass | The IMechE HiL model lists a 279 kg body plus four 5.3 kg wheel carriers and four 6.2 kg wheels. The aggregate model mass is 325 kg. The assembled team-car mass is still needed. |
| CG and inertia | The same nine HiL bodies give a CG 0.76474 m behind the front axle and 0.76526 m ahead of the rear axle, 0.16472 m above the model ground datum. The parallel-axis theorem gives 94.6578 kg·m² for yaw. An in-engine yaw impulse is used to calibrate the CARLA asset's yaw inertia scale. The reference CG is transferred as an offset from the measured CARLA asset COM. |
| Differential and powertrain | The Unreal vehicle has an open rear differential. The supplied steering and traction configuration uses the native drivetrain and preserves grip on both axles. The Saietta 119R-68 is modeled with a 55.7 N·m peak, 4,000 RPM ceiling, and a single 3.5:1 reduction. The HiL shaft map is limited to eight PhysX points by removing the collinear 2,800 RPM point. Torque at and above 1,400 RPM is scaled by 4,999 / 5,345.071 to keep the interpolated shaft-power bound below 5 kW. This is a ceiling based on the vehicle electrical rating. The measured Sevcon Gen4 DC map, efficiency, and regen calibration are unavailable. |
| Battery | The vehicle sheet identifies a four-module lead-acid pack, 100 Ah, nominal 50 V, operating at 48–52 V, and 100 A continuous / 200 A peak discharge. CARLA uses the power ceiling. Pack sag, current transients, and state-dependent torque limits lack vehicle calibration data. |
| Tires | The published 225/45R13 84V Toyo R888R listing gives 616 rev/km. That converts to a 0.258368 m effective rolling radius for the simulator. Nominal outside diameter and measured radius at the team's load and pressure are separate quantities. The HiL reference tire is a generated 185/50R13 model, so its stiffness and rolling resistance are unsuitable as Toyo calibration. |
| Drag | The IMechE HiL model uses Cd 0.467 and frontal area 0.49 m². CARLA uses both, equivalent to CdA 0.22883 m². |
| Maximum steering and speed | The current 2026 DBC specifies a ±30° virtual bicycle request. The CARLA steering scale is 30° at the inside front wheel, with the outer wheel following the chassis's Ackermann geometry. These limits use different angle definitions. The public HiL rack table reaches about ±27.61° virtual at ±48 mm. The physical linkage limits still need a team measurement. The keyboard reaches the full signed range in 1.5 s. The manual accelerator cutoff and both ROS speed target paths use the requested 30 m/s ceiling. |
| Steering feedback | The VCU CAN angle field and steering request use 0.1° per count. The distinct diagnostic actual and target fields use 0.5° per count. ROS per-wheel JointState positions now use radians with the ROS sign convention. The scalar steering feedback stays in degrees and reports the front-wheel average. |
| Actuator and hydraulic dynamics | The vehicle identifies a DC Electronics Microsteer and Shiftec ZE102-1. The documents do not give an ADS-DV steering actuator rate map, lag, or pressure-to-deceleration calibration. The keyboard request slope is 20°/s with the new range. CARLA measurements characterize the simulation and cannot establish those hardware parameters. |
| Alignment and tire forces | The specification lists 6° caster and 2° static camber without a camber sign. The candidate has no new measured cornering stiffness, tire-road friction, rolling resistance, suspension compliance, or combined-slip model. Source tire data or instrumented vehicle measurements are needed to identify them. |

Source material

- [IMechE 2026 dimensions and component specs](https://github.com/FS-AI/FS-AI_ADS-DV_Documentation/blob/bb8f162/FS-AI%20ADS-DV%20Specifications%20%26%20Dimensions.md)
- [IMechE 2026 CAN database](https://github.com/FS-AI/FS-AI_ADS-DV_Documentation/blob/bb8f162/ADS-DV_2026_CAN1.dbc)
- [IMechE 2026 HiL car file](https://github.com/FS-AI/FS-AI_IMechE_ADS-DV_HiL/blob/1fe77bed411ba621a3187bc2eb9ca13c7d371832/Data/Vehicle/FS-AI_IMechE_ADS-DV_2026_HiL)
- [IMechE public CAD reference](https://github.com/FS-AI/FS-AI_ADS-DV_CAD/tree/958fdbf336fcc6cbd60535c2e7812d51a977d267)
- [IMechE HiL rack kinematics](https://github.com/FS-AI/FS-AI_IMechE_ADS-DV_HiL/blob/1fe77bed411ba621a3187bc2eb9ca13c7d371832/Data/Chassis/FS_Autonomous_front.skc)
- [Toyo R888R dimensions](https://www.toyotires.ca/fr/pneus/proxes-r888r)
- [Saietta 119R-68 component datasheet](https://faktor.de/out/media/BR-200-119-R-68.pdf)

## Build and verify

1. Audit a loaded FSAI asset's wheel-bone locations, COM, body mass, and inertia. Run the Unreal authoring script with the audit and the specific published HiL car file. The optional yaw measurement input is the previous asset scale and its measured effective yaw inertia.
2. Cook the changed chassis and front-wheel assets for WindowsNoEditor and LinuxNoEditor. Repack only those cooked assets into the verified baseline runtimes. Keep the original shipping executables and validate all map pack dependencies.
3. Build the stack patch with the matching rebuilt ROS bridge. Supply the bridge executable to the channel packager. Its startup hash is recorded with the packaged physics config and the intended simulator cook.
4. Run the keyboard timing check, real wheel-angle and ROS feedback sweeps, yaw impulse and longitudinal tests using the new build. Confirm the map and platform on each test. Keep maximum achievable speed and stopping distance tied to the tested conditions.
5. Publish to Testing and verify its runtime and stack hashes. Promote only after review.
