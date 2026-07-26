# City RF material tags

This inventory applies to `city_3uav.world`. Only world-instance names are
changed; every `model://` URI remains unchanged. Braced ID lists expand to one
mapping per listed instance (for example, `apartment_{76,86}` means
`apartment_76` and `apartment_86`).

| Original entity name | New entity name | RF material | Reason |
|---|---|---|---|
| `apartment_{76,86}` | `apartment_concrete_{76,86}` | concrete | Multi-storey masonry apartment mesh |
| `fast_food_93` | `fast_food_concrete_93` | concrete | Solid commercial restaurant building |
| `gas_station_73` | `gas_station_concrete_73` | concrete | Solid service-station building/canopy assembly |
| `law_office_{82,155}` | `law_office_concrete_{82,155}` | concrete | Solid commercial office shells |
| `osrf_first_office_87` | `osrf_first_office_concrete_87` | concrete | Solid office building |
| `post_office_143` | `post_office_concrete_143` | concrete | Solid civic/commercial building |
| `salon_{84,154}` | `salon_concrete_{84,154}` | concrete | Texture inspection shows a predominantly concrete storefront, not a glass curtain wall |
| `thrift_shop_83` | `thrift_shop_concrete_83` | concrete | Texture inspection shows a predominantly brick storefront |
| `house_1_{66,67,146}` | `house_1_wood_{66,67,146}` | wood | Timber/siding residential house mesh |
| `house_2_{71,125,126}` | `house_2_wood_{71,125,126}` | wood | Residential variant of the same timber house family |
| `house_3_{68,156,157,158}` | `house_3_wood_{68,156,157,158}` | wood | Residential variant of the same timber house family |
| `oak_tree_{44-55,69,70,77-80,85,91,148,150}` | `oak_tree_foliage_{same IDs}` | foliage | Dense broadleaf tree meshes |
| `pine_tree_{56-65,88-90,127-142,145,149,151,159-165}` | `pine_tree_foliage_{same IDs}` | foliage | Dense conifer tree meshes |
| `ambulance_72` | `ambulance_vehicle_72` | vehicle | Large parked road vehicle |
| `hatchback_{247,250}` | `hatchback_vehicle_{247,250}` | vehicle | Parked cars |
| `hatchback_blue_{241,245,252}` | `hatchback_blue_vehicle_{241,245,252}` | vehicle | Parked cars |
| `hatchback_red_75` | `hatchback_red_vehicle_75` | vehicle | Parked car |
| `pickup_{147,242,244,246,248,249,253}` | `pickup_vehicle_{same IDs}` | vehicle | Parked pickup trucks |
| `suv_{243,251,254}` | `suv_vehicle_{243,251,254}` | vehicle | Parked large cars |
| `dumpster_94` | `dumpster_metal_94` | metal | Large steel refuse container |
| `radio_tower_124` | `radio_tower_metal_124` | metal | Steel lattice radio tower |
| `truss_bridge_{167,168}` | `truss_bridge_metal_{167,168}` | metal | Steel truss structures |
| `fire_hydrant_{81,95,166}` | `fire_hydrant_noloss_{81,95,166}` | noloss | Small street fixtures |
| `lamp_post_{189-198}` | `lamp_post_noloss_{189-198}` | noloss | Thin poles that should not shadow an RF link |
| `stop_light_post_{169-182}` | `stop_light_post_noloss_{169-182}` | noloss | Thin traffic-light structures |
| `stop_sign_{183-188}` | `stop_sign_noloss_{183-188}` | noloss | Thin signs |
| `telephone_pole_{199-240}` | `telephone_pole_noloss_{199-240}` | noloss | Thin utility poles |
| `postbox_{92,144}` | `postbox_noloss_{92,144}` | noloss | Small street furniture |
| `city_terrain_1` | `city_terrain_noloss_1` | noloss | Terrain surface must not mask buildings behind it |
| `asphalt_plane_74` | `asphalt_plane_noloss_74` | noloss | Road surface must not attenuate or mask links |
| `sidewalk_{3-40}` | `sidewalk_noloss_{3-40}` | noloss | Low ground-adjacent surfaces must not mask real obstacles |

## Deliberately unchanged

- `ground_plane` is already explicitly filtered by the plugin.
- `iris_1`, `iris_2`, `iris_3`, and `gcs` retain the integrated node naming
  convention and are explicitly filtered by the plugin.
- `ocean_2` has no useful solid RF obstruction.
- `fountain_41`, `gazebo_42`, `pier_43`, and `cardboard_box_152` remain
  RF-transparent: they are decorative or compositionally ambiguous and are
  not major city blockers.
- `target_pond_*` and `target_mountain_*` are detection targets, not RF
  obstacles.
- Roads and visual-only scenery without collision entity names are unchanged.

No entity is tagged `glass`. The inspected commercial textures contain
windows, but none of the collision meshes represents a predominantly
glass-fronted or glass-curtain-wall structure. Tagging an entire masonry
building as glass would understate its loss.
