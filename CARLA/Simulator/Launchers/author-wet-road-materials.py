"""Apply the reviewed ASURT wet-road material in a UE4.26 editor cook project.

Set ASURT_WET_ROAD_MASK to the file written by generate-wet-road-mask.py.
"""

import os

import unreal


MATERIALS = (
    ("/Game/Carla/Maps/M_FlatAsphalt", 0.54, True),
    ("/Game/ASURTMaps/silverstone_section/silverstoneSplit/Materials/Master/asph_pitlane_001", 0.32, False),
)
WEATHER_COLLECTION = "/Game/Carla/Blueprints/Weather/Materials/WeatherMaterialParameters"
FLAT_TEXTURE_FOLDER = "/Game/Carla/Maps"
SILVERSTONE_TEXTURE_FOLDER = "/Game/ASURTMaps/silverstone_section/silverstoneSplit/Materials"
TEXTURE_NAME = "T_ASURT_WetAsphaltPattern"
PREVIOUS_DESCRIPTION = "ASURT road water from CARLA weather"
DESCRIPTION = "ASURT wet asphalt with a shared surface mask"

assets = unreal.EditorAssetLibrary
editing = unreal.MaterialEditingLibrary


def custom_input(name):
    pin = unreal.CustomInput()
    pin.set_editor_property("input_name", name)
    return pin


def connect(source, expression, name):
    if not editing.connect_material_expressions(source, "", expression, name):
        raise ValueError(f"Could not connect {name} on {expression.get_path_name()}")


def load_mask(folder):
    path = f"{folder}/{TEXTURE_NAME}"
    if not assets.does_asset_exist(path):
        task = unreal.AssetImportTask()
        task.set_editor_property("filename", os.environ["ASURT_WET_ROAD_MASK"])
        task.set_editor_property("destination_path", folder)
        task.set_editor_property("destination_name", TEXTURE_NAME)
        task.set_editor_property("automated", True)
        unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    texture = assets.load_asset(path)
    if not isinstance(texture, unreal.Texture2D):
        raise ValueError(f"Unable to load the asphalt mask at {path}")
    texture.set_editor_property("srgb", False)
    texture.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_MASKS)
    texture.set_editor_property("compression_no_alpha", True)
    if not assets.save_loaded_asset(texture):
        raise RuntimeError(f"Could not save {path}")
    return texture


def get_weather_expression(material, prop, output_type, collection):
    current = editing.get_material_property_input_node(material, prop)
    if not current:
        raise ValueError(f"Missing {prop} on {material.get_path_name()}")
    names = ("Dry", "Wetness", "Puddles")
    if current.get_editor_property("desc") == PREVIOUS_DESCRIPTION:
        inputs = current.get_editor_property("inputs")
        sources = editing.get_inputs_for_material_expression(material, current)
        if tuple(str(pin.get_editor_property("input_name")) for pin in inputs) != names or len(sources) != len(names):
            raise ValueError(f"Unexpected weather inputs on {current.get_path_name()}")
        return current, dict(zip(names, sources))
    expression = editing.create_material_expression(material, unreal.MaterialExpressionCustom, 420, 0)
    expression.set_editor_property("output_type", output_type)
    sources = {"Dry": current}
    for index, name in enumerate(names[1:]):
        parameter = editing.create_material_expression(
            material, unreal.MaterialExpressionCollectionParameter, 0, 240 + 140 * index
        )
        parameter.set_editor_property("collection", collection)
        parameter.set_editor_property("parameter_name", name)
        sources[name] = parameter
    if not editing.connect_material_property(expression, "", prop):
        raise ValueError(f"Could not connect {prop} on {material.get_path_name()}")
    return expression, sources


def shading(grain, flat):
    edge_width = "0.15" if flat else "0.09"
    water = f"""
float w = saturate(Wetness);
float p = saturate(Puddles);
float film = saturate(max(w, p * 0.74) * lerp(0.87, 1.33, Pattern.b));
float edge = lerp(0.98, 0.40, p);
float pool = smoothstep(0.02, 0.68, p) * smoothstep(edge, edge + {edge_width}, Pattern.r);
"""
    if flat:
        coverage = """
float distantFilm = 0.23 * smoothstep(0.02, 0.68, p);
float localWater = lerp(distantFilm, pool, smoothstep(0.022, 0.18, angle));
"""
        tint = "0.052, 0.067, 0.084"
    else:
        coverage = "float localWater = pool;"
        tint = "0.070, 0.091, 0.115"
    color = water + f"""
float micrograin = 1.0 + {grain:.3f} * (Pattern.g - 0.5);
float3 soaked = Dry * lerp(0.16, 0.31, Pattern.b) * micrograin * float3(0.85, 0.94, 1.05);
float angle = abs(View.z);
float grazing = saturate(1.0 - angle);
{coverage}
float3 sheen = grazing * grazing * float3({tint});
return lerp(Dry, soaked, film) + sheen * localWater;
"""
    roughness = water + """
float dampRoughness = lerp(0.48, 0.63, Pattern.g);
float poolRoughness = lerp(0.28, 0.37, Pattern.b);
return lerp(lerp(Dry, dampRoughness, film), poolRoughness, pool);
"""
    return color, roughness


def update_material(path, grain, flat, collection, texture):
    material = assets.load_asset(path)
    if not isinstance(material, unreal.Material):
        raise ValueError(f"Expected an editable material at {path}")
    color = unreal.MaterialProperty.MP_BASE_COLOR
    roughness = unreal.MaterialProperty.MP_ROUGHNESS
    if all(editing.get_material_property_input_node(material, prop).get_editor_property("desc") == DESCRIPTION
           for prop in (color, roughness)):
        unreal.log(f"Cook-ready asphalt weather material at {path}")
        return

    world = editing.create_material_expression(material, unreal.MaterialExpressionWorldPosition, -610, 700)
    uv = editing.create_material_expression(material, unreal.MaterialExpressionCustom, -340, 700)
    uv.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT2)
    uv.set_editor_property("code", "return P.xy / float2(2860.0, 1580.0);")
    uv.set_editor_property("inputs", [custom_input("P")])
    connect(world, uv, "P")
    sample = editing.create_material_expression(material, unreal.MaterialExpressionTextureSample, -110, 700)
    sample.set_editor_property("texture", texture)
    sample.set_editor_property("sampler_type", unreal.MaterialSamplerType.SAMPLERTYPE_MASKS)
    connect(uv, sample, "")
    camera = editing.create_material_expression(material, unreal.MaterialExpressionCameraVectorWS, -110, 920)

    color_code, roughness_code = shading(grain, flat)
    outputs = (
        (color, unreal.CustomMaterialOutputType.CMOT_FLOAT3, color_code, {"Pattern": sample, "View": camera}),
        (roughness, unreal.CustomMaterialOutputType.CMOT_FLOAT1, roughness_code, {"Pattern": sample}),
    )
    for prop, output_type, code, extra in outputs:
        expression, inputs = get_weather_expression(material, prop, output_type, collection)
        inputs.update(extra)
        expression.set_editor_property("inputs", [custom_input(name) for name in inputs])
        for name, source in inputs.items():
            connect(source, expression, name)
        expression.set_editor_property("code", code)
        expression.set_editor_property("desc", DESCRIPTION)
    editing.recompile_material(material)
    if not assets.save_loaded_asset(material):
        raise RuntimeError(f"Could not save {path}")
    unreal.log(f"Cook-ready asphalt weather material at {path}")


collection = assets.load_asset(WEATHER_COLLECTION)
if not isinstance(collection, unreal.MaterialParameterCollection):
    raise ValueError(f"Missing CARLA weather collection {WEATHER_COLLECTION}")
for material_path, grain_strength, is_flat in MATERIALS:
    folder = FLAT_TEXTURE_FOLDER if is_flat else SILVERSTONE_TEXTURE_FOLDER
    update_material(material_path, grain_strength, is_flat, collection, load_mask(folder))
