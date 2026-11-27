"""Run in a UE4.26 editor cook project to update the shipped asphalt materials."""

import unreal


MATERIALS = (
    "/Game/Carla/Maps/M_FlatAsphalt",
    "/Game/ASURTMaps/silverstone_section/silverstoneSplit/Materials/Master/asph_pitlane_001",
)
WEATHER_COLLECTION = "/Game/Carla/Blueprints/Weather/Materials/WeatherMaterialParameters"
DESCRIPTION = "ASURT road water from CARLA weather"

assets = unreal.EditorAssetLibrary
editing = unreal.MaterialEditingLibrary


def update_material(path, collection):
    material = assets.load_asset(path)
    if not isinstance(material, unreal.Material):
        raise ValueError(f"Expected an editable material at {path}")
    color = unreal.MaterialProperty.MP_BASE_COLOR
    roughness = unreal.MaterialProperty.MP_ROUGHNESS
    dry_color = editing.get_material_property_input_node(material, color)
    dry_roughness = editing.get_material_property_input_node(material, roughness)
    if not dry_color or not dry_roughness:
        raise ValueError(f"Missing the original asphalt shading inputs on {path}")
    if dry_color.get_editor_property("desc") == DESCRIPTION:
        return

    parameters = {}
    for index, name in enumerate(("Wetness", "Puddles")):
        parameter = editing.create_material_expression(
            material, unreal.MaterialExpressionCollectionParameter, 0, 240 + 140 * index
        )
        parameter.set_editor_property("collection", collection)
        parameter.set_editor_property("parameter_name", name)
        parameters[name] = parameter

    outputs = (
        (color, dry_color, unreal.CustomMaterialOutputType.CMOT_FLOAT3,
         "return Dry * lerp(1.0, 0.57, saturate(max(Wetness, Puddles)));"),
        (roughness, dry_roughness, unreal.CustomMaterialOutputType.CMOT_FLOAT1,
         "return lerp(Dry, lerp(0.21, 0.09, saturate(Puddles)), saturate(max(Wetness, Puddles)));"),
    )
    for index, (prop, dry, output_type, code) in enumerate(outputs):
        expression = editing.create_material_expression(
            material, unreal.MaterialExpressionCustom, 420, -80 + 180 * index
        )
        expression.set_editor_property("desc", DESCRIPTION)
        expression.set_editor_property("output_type", output_type)
        expression.set_editor_property("code", code)
        inputs = []
        for name in ("Dry", *parameters):
            custom_input = unreal.CustomInput()
            custom_input.set_editor_property("input_name", name)
            inputs.append(custom_input)
        expression.set_editor_property("inputs", inputs)
        for name, source in (("Dry", dry), *parameters.items()):
            if not editing.connect_material_expressions(source, "", expression, name):
                raise ValueError(f"Could not connect {name} on {path}")
        if not editing.connect_material_property(expression, "", prop):
            raise ValueError(f"Could not connect {prop} on {path}")

    editing.recompile_material(material)
    if not assets.save_loaded_asset(material):
        raise RuntimeError(f"Could not save {path}")
    unreal.log(f"Cook-ready asphalt weather material at {path}")


collection = assets.load_asset(WEATHER_COLLECTION)
if not isinstance(collection, unreal.MaterialParameterCollection):
    raise ValueError(f"Missing CARLA weather collection {WEATHER_COLLECTION}")
for material_path in MATERIALS:
    update_material(material_path, collection)
