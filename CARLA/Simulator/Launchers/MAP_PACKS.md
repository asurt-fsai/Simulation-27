# FlatAsphalt and add-on maps

The editable CarlaUE4 project opens FlatAsphalt by default. Its packaging list contains FlatAsphalt. The dedicated `cook-flat-asphalt.ps1` workflow uses the trimmed FSAI and cone factories in its cook workspace. It also cooks the saved vehicle physics, map collision, sensor dependencies and weather materials. The staging script uses the dependency report to exclude stock scenery loaded by editor tooling and includes the classes CARLA loads at startup.

Save each add-on map and its unique assets under `/Game/ASURTMaps/<id_with_underscores>/`. For example, put `SilverstoneNational` directly in `/Game/ASURTMaps/silverstone_national/`. Keep shared assets in the base build. Give each map its own OpenDRIVE file and verify a spawn pose in CARLA coordinates.

From this launcher folder, cook the new map for the matching FlatAsphalt build.

```powershell
.\cook-map-pack.ps1 -Workspace 'E:\ASURT' -Id silverstone-national -Label 'Silverstone National' `
  -Map /Game/ASURTMaps/silverstone_national/SilverstoneNational `
  -OpenDrive 'E:\path\SilverstoneNational.xodr' -Spawn @(10, 20, 0.8, 90) `
  -Destination 'E:\path\SilverstoneNationalCook'
```

`-Spawn` takes X, Y, Z in metres followed by yaw in degrees. Supply the map's real pose. `-Platforms` can select just WindowsNoEditor or LinuxNoEditor. The command outputs a ZIP containing the map's cooked files and its road data. The map pack verifies the CARLA Shipping executable and the shared assets it reuses. Use `-WindowsBase` or `-LinuxBase` when your intended FlatAsphalt base differs from the active build.

Open **ASURT CARLA Map Packs** or **Map Packs** in the System Manager. Use **Add Map Pack** to register the ZIP, choose the available runtime profile, then select **Load Map**. Switching maps within a profile replaces the current simulator session and attaches the tools and sensor stack to the new one.

**Silverstone Split** uses the `silverstone-section` pack ID and the level at `/Game/ASURTMaps/silverstone_section/SilverStoneSection`. Its OpenDRIVE describes the cropped main route, with open ends at the crop boundary. Cone layouts can be imported with Track Importer from the active map's course origin. To return to the training pad, select **FlatAsphalt** and click **Load Map**.
