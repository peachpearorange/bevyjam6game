#![feature(let_chains)]
#![allow(dead_code)]

use {bevy::{asset::AssetMetaCheck, prelude::*},
     bevy_mod_billboard::prelude::*};
// use bevy::math::Affine2; // Not needed with voxel world
use {avian3d::prelude::*,
     bevy::input::mouse::{MouseMotion, MouseWheel}};
// use avian3d::collision::AsyncCollider; // Not available in this version
use {bevy_embedded_assets::EmbeddedAssetPlugin, bevy_voxel_world::prelude::*,
     enum_assoc::Assoc};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Assoc)]
#[func(const fn texture_coords(self) -> u32)]
enum BlockType {
  #[assoc(texture_coords = 0)]
  Grass,
  #[assoc(texture_coords = 1)]
  Road,
  #[assoc(texture_coords = 2)]
  Sidewalk,
  #[assoc(texture_coords = 3)]
  Building,
  #[assoc(texture_coords = 4)]
  Window,
  #[assoc(texture_coords = 5)]
  Door,
  #[assoc(texture_coords = 6)]
  Roof,
  #[assoc(texture_coords = 7)]
  RedBricks,
  #[assoc(texture_coords = 8)]
  GrayBricks
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BuildingType {
  Residential,
  Commercial,
  Industrial,
  Office,
  Mixed
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DistrictType {
  Downtown,
  Residential,
  Industrial,
  Commercial,
  Park
}

impl BlockType {
  const TEXTURE_SIZE: u32 = 16;
  const SIDE_LENGTH_PIXELS: u32 = 10;

  fn get_texture_atlas_index(self) -> usize { self.texture_coords() as usize }
}

const CITY_SIZE: i32 = 40;
const BLOCK_SIZE: f32 = 1.0;
const BUILDING_HEIGHT_MIN: i32 = 5;
const BUILDING_HEIGHT_MAX: i32 = 35;
const MAIN_ROAD_WIDTH: i32 = 3;
const SIDE_ROAD_WIDTH: i32 = 1;
const DISTRICT_SIZE: i32 = 40;

#[derive(Component)]
struct Player;

#[derive(Component)]
struct Pizza;

#[derive(Component)]
struct PizzaTimer {
  timer: Timer
}

impl PizzaTimer {
  fn new() -> Self { Self { timer: Timer::from_seconds(5.0, TimerMode::Once) } }
}

#[derive(Component)]
struct CityBlock {
  block_type: BlockType,
  position: IVec3
}

#[derive(Component)]
struct Tree;

#[derive(Component)]
struct Npc {
  wants_pizza: bool,
  satisfied: bool
}

// SpriteFollower component removed - entities are now self-contained

#[derive(Resource, Clone, Default)]
struct MyWorld;

impl VoxelWorldConfig for MyWorld {
  type MaterialIndex = u8;
  type ChunkUserBundle = (RigidBody, Collider, Friction, Restitution);

  fn spawning_distance(&self) -> u32 { 100 }

  fn texture_index_mapper(&self) -> std::sync::Arc<dyn Fn(u8) -> [u32; 3] + Send + Sync> {
    std::sync::Arc::new(|material_index: u8| {
      // Return [top, sides, bottom] texture indices
      // For our game, we'll use the same texture for all faces
      [material_index as u32, material_index as u32, material_index as u32]
    })
  }

  fn voxel_texture(&self) -> Option<(String, u32)> {
    // blocks.png texture with 10 different textures stacked vertically
    Some(("blocks.png".into(), 10))
  }
}

#[derive(Resource)]
struct GameAssets {
  player_texture: Handle<Image>,
  pizza_texture: Handle<Image>,
  tree_texture: Handle<Image>,
  woman1_texture: Handle<Image>,
  ducky_texture: Handle<Image>,
  blocks_layout: Handle<TextureAtlasLayout>,
  block_material: Handle<StandardMaterial>,
  quad_mesh: Handle<Mesh>,
  npc_mesh: Handle<Mesh>,
  player_mesh: Handle<Mesh>,
  tree_mesh: Handle<Mesh>,
  pizza_mesh: Handle<Mesh>
}

#[derive(Resource)]
struct PlayerStats {
  pizzas_delivered: u32,
  pizzas_remaining: u32
}

#[derive(Component)]
struct CameraController {
  target: Vec3,
  distance: f32,
  pitch: f32,
  yaw: f32
}

fn main() {
  App::new()
    .add_plugins(
      DefaultPlugins
        .set(AssetPlugin { meta_check: AssetMetaCheck::Never, ..default() })
        .set(ImagePlugin::default_nearest())
    )
    .add_plugins(EmbeddedAssetPlugin::default())
    .add_plugins(PhysicsPlugins::default())
    .add_plugins(VoxelWorldPlugin::with_config(MyWorld))
    .add_plugins(BillboardPlugin)
    .insert_resource(PlayerStats { pizzas_delivered: 0, pizzas_remaining: 20 })
    .add_systems(
      Startup,
      (setup_assets, setup_camera, setup_voxel_world, generate_city, spawn_player).chain()
    )
    .add_systems(
      Update,
      (
        // player_movement,
        // player_jump,
        player_move_and_jump,
        throw_pizza,
        pizza_physics,
        pizza_timer_system,
        pizza_delivery_system,
        // sprite_follow_system, // Removed - no longer needed
        camera_follow_player,
        camera_controller,
        mouse_look
      )
    )
    .add_systems(Update, (add_colliders_to_chunks, update_ui))
    .run();
}

fn setup_assets(
  mut commands: Commands,
  asset_server: Res<AssetServer>,
  mut texture_atlas_layouts: ResMut<Assets<TextureAtlasLayout>>,
  mut materials: ResMut<Assets<StandardMaterial>>,
  mut meshes: ResMut<Assets<Mesh>>
) {
  // Don't load blocks_texture here since voxel world handles it
  let player_texture = asset_server.load("guy.png");
  let pizza_texture = asset_server.load("pizzabox.png");
  let tree_texture = asset_server.load("tree.png");
  let woman1_texture = asset_server.load("woman1.png");
  let ducky_texture = asset_server.load("ducky.png");

  // Remove texture atlas layout since voxel world handles textures
  let blocks_layout =
    texture_atlas_layouts.add(TextureAtlasLayout::new_empty(UVec2::splat(10)));

  // Remove block_material since voxel world handles its own materials
  let block_material = materials.add(StandardMaterial::default());

  let quad_mesh = meshes.add(Rectangle::new(1.0, 1.0));
  let npc_mesh = meshes.add(Rectangle::new(1.0, 2.0));
  let player_mesh = meshes.add(Rectangle::new(1.0, 2.0));
  let tree_mesh = meshes.add(Rectangle::new(1.5, 2.0));
  let pizza_mesh = meshes.add(Rectangle::new(0.8, 0.8)); // Slightly smaller for visibility

  commands.insert_resource(GameAssets {
    player_texture,
    pizza_texture,
    tree_texture,
    woman1_texture,
    ducky_texture,
    blocks_layout,
    block_material,
    quad_mesh,
    npc_mesh,
    player_mesh,
    tree_mesh,
    pizza_mesh
  });
}

fn setup_camera(mut commands: Commands) {
  commands.spawn((
    Camera3d::default(),
    Transform::from_xyz(25.0, 15.0, 25.0).looking_at(Vec3::ZERO, Vec3::Y),
    VoxelWorldCamera::<MyWorld>::default(),
    CameraController { target: Vec3::ZERO, distance: 30.0, pitch: -0.5, yaw: -0.5 }
  ));

  commands.spawn((
    DirectionalLight { illuminance: 10000.0, shadows_enabled: true, ..default() },
    Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.5, -0.5, 0.0))
  ));
}

fn setup_voxel_world() {
  // Voxel world is automatically set up by the plugin
}

fn get_district_type(x: i32, z: i32) -> DistrictType {
  let district_x = x / DISTRICT_SIZE;
  let district_z = z / DISTRICT_SIZE;

  // Create a deterministic pattern based on district coordinates
  let pattern = (district_x + district_z * 3).abs() % 5;
  match pattern {
    0 => DistrictType::Downtown,
    1 => DistrictType::Residential,
    2 => DistrictType::Commercial,
    3 => DistrictType::Industrial,
    _ => DistrictType::Park
  }
}

fn get_building_type_for_district(
  district: DistrictType,
  rng: &mut dyn FnMut() -> u32
) -> BuildingType {
  match district {
    DistrictType::Downtown => match rng() % 3 {
      0 => BuildingType::Office,
      1 => BuildingType::Commercial,
      _ => BuildingType::Mixed
    },
    DistrictType::Residential => BuildingType::Residential,
    DistrictType::Commercial => BuildingType::Commercial,
    DistrictType::Industrial => BuildingType::Industrial,
    DistrictType::Park => BuildingType::Residential // Sparse low buildings
  }
}

fn get_building_params(
  building_type: BuildingType,
  district: DistrictType,
  rng: &mut dyn FnMut() -> u32
) -> (i32, i32, i32, f32) {
  match building_type {
    BuildingType::Residential => {
      let width = 4 + (rng() % 6) as i32; // 4-9 blocks
      let depth = 4 + (rng() % 6) as i32;
      let height = match district {
        DistrictType::Downtown => 15 + (rng() % 15) as i32, // 15-29 floors
        DistrictType::Park => 3 + (rng() % 4) as i32,       // 3-6 floors
        _ => 8 + (rng() % 12) as i32                        // 8-19 floors
      };
      (width, depth, height, 0.8) // 80% spawn chance
    }
    BuildingType::Commercial => {
      let width = 6 + (rng() % 10) as i32; // 6-15 blocks  
      let depth = 6 + (rng() % 10) as i32;
      let height = match district {
        DistrictType::Downtown => 20 + (rng() % 15) as i32, // 20-34 floors
        _ => 5 + (rng() % 10) as i32                        // 5-14 floors
      };
      (width, depth, height, 0.9) // 90% spawn chance
    }
    BuildingType::Industrial => {
      let width = 8 + (rng() % 15) as i32; // 8-22 blocks
      let depth = 8 + (rng() % 15) as i32;
      let height = 3 + (rng() % 6) as i32; // 3-8 floors (low and wide)
      (width, depth, height, 0.7) // 70% spawn chance
    }
    BuildingType::Office => {
      let width = 5 + (rng() % 8) as i32; // 5-12 blocks
      let depth = 5 + (rng() % 8) as i32;
      let height = match district {
        DistrictType::Downtown => 25 + (rng() % 10) as i32, // 25-34 floors (skyscrapers)
        _ => 12 + (rng() % 15) as i32                       // 12-26 floors
      };
      (width, depth, height, 0.85) // 85% spawn chance
    }
    BuildingType::Mixed => {
      let width = 5 + (rng() % 8) as i32;
      let depth = 5 + (rng() % 8) as i32;
      let height = 10 + (rng() % 20) as i32; // 10-29 floors
      (width, depth, height, 0.9) // 90% spawn chance
    }
  }
}

fn is_main_road(x: i32, z: i32) -> bool {
  // Major arterials every 30 blocks
  (x % 30 == 0) || (z % 30 == 0) ||
  // Secondary roads every 15 blocks
  (x % 15 == 0) || (z % 15 == 0)
}

fn get_road_width(x: i32, z: i32) -> i32 {
  if (x % 30 == 0) || (z % 30 == 0) { MAIN_ROAD_WIDTH } else { SIDE_ROAD_WIDTH }
}

#[derive(Debug, Clone, Copy)]
enum BuildingShape {
  Simple,
  Tower,
  Stepped,
  LShape,
  Courtyard,
  CrossShape,
  TShape,
  MultipleTowers,
  Curved,
  Ziggurat
}

fn place_building_block(
  voxel_world: &mut VoxelWorld<MyWorld>,
  x: i32,
  y: i32,
  z: i32,
  building_type: BuildingType,
  is_edge: bool,
  is_top: bool,
  rng: &mut dyn FnMut() -> u32
) {
  let block_type = if is_top {
    BlockType::Roof
  } else if y == 1 && is_edge && (rng() % 8) == 0 {
    BlockType::Door
  } else if is_edge && (rng() % 3) == 0 {
    BlockType::Window
  } else if is_edge {
    match building_type {
      BuildingType::Industrial => BlockType::GrayBricks,
      BuildingType::Residential => {
        if (rng() % 2) == 0 {
          BlockType::RedBricks
        } else {
          BlockType::Building
        }
      }
      _ => BlockType::Building
    }
  } else {
    BlockType::Building
  };

  voxel_world
    .set_voxel(IVec3::new(x, y, z), WorldVoxel::Solid(block_type.texture_coords() as u8));
}

fn generate_complex_building(
  voxel_world: &mut VoxelWorld<MyWorld>,
  occupied: &mut std::collections::HashMap<(i32, i32), bool>,
  start_x: i32,
  start_z: i32,
  width: i32,
  depth: i32,
  height: i32,
  building_type: BuildingType,
  rng: &mut dyn FnMut() -> u32
) {
  // Test with just a few shapes to debug
  let shape = match rng() % 5 {
    0 => BuildingShape::Simple,
    1 => BuildingShape::Tower,
    2 => BuildingShape::LShape,
    3 => BuildingShape::CrossShape,
    _ => BuildingShape::Stepped
  };

  println!("Generating building: {:?} shape at ({}, {})", shape, start_x, start_z);

  match shape {
    BuildingShape::Simple => generate_simple_building(
      voxel_world,
      occupied,
      start_x,
      start_z,
      width,
      depth,
      height,
      building_type,
      rng
    ),
    BuildingShape::Tower => generate_tower_building(
      voxel_world,
      occupied,
      start_x,
      start_z,
      width,
      depth,
      height,
      building_type,
      rng
    ),
    BuildingShape::Stepped => generate_stepped_building(
      voxel_world,
      occupied,
      start_x,
      start_z,
      width,
      depth,
      height,
      building_type,
      rng
    ),
    BuildingShape::LShape => generate_l_shaped_building(
      voxel_world,
      occupied,
      start_x,
      start_z,
      width,
      depth,
      height,
      building_type,
      rng
    ),
    BuildingShape::Courtyard => generate_courtyard_building(
      voxel_world,
      occupied,
      start_x,
      start_z,
      width,
      depth,
      height,
      building_type,
      rng
    ),
    BuildingShape::CrossShape => generate_cross_building(
      voxel_world,
      occupied,
      start_x,
      start_z,
      width,
      depth,
      height,
      building_type,
      rng
    ),
    BuildingShape::TShape => generate_t_building(
      voxel_world,
      occupied,
      start_x,
      start_z,
      width,
      depth,
      height,
      building_type,
      rng
    ),
    BuildingShape::MultipleTowers => generate_multiple_towers_building(
      voxel_world,
      occupied,
      start_x,
      start_z,
      width,
      depth,
      height,
      building_type,
      rng
    ),
    BuildingShape::Curved => generate_curved_building(
      voxel_world,
      occupied,
      start_x,
      start_z,
      width,
      depth,
      height,
      building_type,
      rng
    ),
    BuildingShape::Ziggurat => generate_ziggurat_building(
      voxel_world,
      occupied,
      start_x,
      start_z,
      width,
      depth,
      height,
      building_type,
      rng
    )
  }
}

fn generate_simple_building(
  voxel_world: &mut VoxelWorld<MyWorld>,
  occupied: &mut std::collections::HashMap<(i32, i32), bool>,
  start_x: i32,
  start_z: i32,
  width: i32,
  depth: i32,
  height: i32,
  building_type: BuildingType,
  rng: &mut dyn FnMut() -> u32
) {
  for bx in start_x..(start_x + width) {
    for bz in start_z..(start_z + depth) {
      occupied.insert((bx, bz), true);

      for y in 1..=height {
        let is_edge = bx == start_x
          || bx == start_x + width - 1
          || bz == start_z
          || bz == start_z + depth - 1;
        let is_top = y == height;

        place_building_block(voxel_world, bx, y, bz, building_type, is_edge, is_top, rng);
      }
    }
  }
}

fn generate_tower_building(
  voxel_world: &mut VoxelWorld<MyWorld>,
  occupied: &mut std::collections::HashMap<(i32, i32), bool>,
  start_x: i32,
  start_z: i32,
  width: i32,
  depth: i32,
  height: i32,
  building_type: BuildingType,
  rng: &mut dyn FnMut() -> u32
) {
  // Base building (70% of height)
  let base_height = (height as f32 * 0.7) as i32;
  generate_simple_building(
    voxel_world,
    occupied,
    start_x,
    start_z,
    width,
    depth,
    base_height,
    building_type,
    rng
  );

  // Tower section (smaller, centered)
  let tower_width = width / 2 + 1;
  let tower_depth = depth / 2 + 1;
  let tower_start_x = start_x + (width - tower_width) / 2;
  let tower_start_z = start_z + (depth - tower_depth) / 2;

  for bx in tower_start_x..(tower_start_x + tower_width) {
    for bz in tower_start_z..(tower_start_z + tower_depth) {
      for y in (base_height + 1)..=height {
        let is_edge = bx == tower_start_x
          || bx == tower_start_x + tower_width - 1
          || bz == tower_start_z
          || bz == tower_start_z + tower_depth - 1;
        let is_top = y == height;

        place_building_block(voxel_world, bx, y, bz, building_type, is_edge, is_top, rng);
      }
    }
  }
}

fn generate_stepped_building(
  voxel_world: &mut VoxelWorld<MyWorld>,
  occupied: &mut std::collections::HashMap<(i32, i32), bool>,
  start_x: i32,
  start_z: i32,
  width: i32,
  depth: i32,
  height: i32,
  building_type: BuildingType,
  rng: &mut dyn FnMut() -> u32
) {
  let steps = 3.min(height / 4);
  let step_height = height / (steps + 1);

  for step in 0..=steps {
    let step_shrink = step * 2;
    let step_width = (width - step_shrink).max(3);
    let step_depth = (depth - step_shrink).max(3);
    let step_start_x = start_x + step_shrink / 2;
    let step_start_z = start_z + step_shrink / 2;
    let step_max_height = height - (step * step_height);
    let step_min_height =
      if step == 0 { 1 } else { height - ((step - 1) * step_height) + 1 };

    for bx in step_start_x..(step_start_x + step_width) {
      for bz in step_start_z..(step_start_z + step_depth) {
        if step == 0 {
          occupied.insert((bx, bz), true);
        }

        for y in step_min_height..=step_max_height {
          let is_edge = bx == step_start_x
            || bx == step_start_x + step_width - 1
            || bz == step_start_z
            || bz == step_start_z + step_depth - 1;
          let is_top = y == step_max_height;

          place_building_block(voxel_world, bx, y, bz, building_type, is_edge, is_top, rng);
        }
      }
    }
  }
}

fn generate_l_shaped_building(
  voxel_world: &mut VoxelWorld<MyWorld>,
  occupied: &mut std::collections::HashMap<(i32, i32), bool>,
  start_x: i32,
  start_z: i32,
  width: i32,
  depth: i32,
  height: i32,
  building_type: BuildingType,
  rng: &mut dyn FnMut() -> u32
) {
  // Create L-shape directly without recursion
  let wing1_width = width;
  let wing1_depth = depth / 2 + 1;
  let wing2_width = width / 2 + 1;
  let wing2_depth = depth - wing1_depth;
  let wing2_start_z = start_z + wing1_depth;

  // Build wing 1 (horizontal bar)
  for bx in start_x..(start_x + wing1_width) {
    for bz in start_z..(start_z + wing1_depth) {
      occupied.insert((bx, bz), true);
      for y in 1..=height {
        let is_edge = bx == start_x
          || bx == start_x + wing1_width - 1
          || bz == start_z
          || bz == start_z + wing1_depth - 1;
        let is_top = y == height;
        place_building_block(voxel_world, bx, y, bz, building_type, is_edge, is_top, rng);
      }
    }
  }

  // Build wing 2 (vertical bar)
  for bx in start_x..(start_x + wing2_width) {
    for bz in wing2_start_z..(wing2_start_z + wing2_depth) {
      if let std::collections::hash_map::Entry::Vacant(e) = occupied.entry((bx, bz)) {
        // Don't override existing blocks
        e.insert(true);
        for y in 1..=height {
          let is_edge = bx == start_x
            || bx == start_x + wing2_width - 1
            || bz == wing2_start_z
            || bz == wing2_start_z + wing2_depth - 1;
          let is_top = y == height;
          place_building_block(voxel_world, bx, y, bz, building_type, is_edge, is_top, rng);
        }
      }
    }
  }
}

fn generate_courtyard_building(
  voxel_world: &mut VoxelWorld<MyWorld>,
  occupied: &mut std::collections::HashMap<(i32, i32), bool>,
  start_x: i32,
  start_z: i32,
  width: i32,
  depth: i32,
  height: i32,
  building_type: BuildingType,
  rng: &mut dyn FnMut() -> u32
) {
  // Only create courtyard if building is large enough
  if width < 8 || depth < 8 {
    generate_simple_building(
      voxel_world,
      occupied,
      start_x,
      start_z,
      width,
      depth,
      height,
      building_type,
      rng
    );
    return;
  }

  // Courtyard dimensions (inner empty space)
  let courtyard_width = width / 3;
  let courtyard_depth = depth / 3;
  let courtyard_start_x = start_x + (width - courtyard_width) / 2;
  let courtyard_start_z = start_z + (depth - courtyard_depth) / 2;

  for bx in start_x..(start_x + width) {
    for bz in start_z..(start_z + depth) {
      // Skip courtyard area
      if bx >= courtyard_start_x
        && bx < courtyard_start_x + courtyard_width
        && bz >= courtyard_start_z
        && bz < courtyard_start_z + courtyard_depth
      {
        continue;
      }

      occupied.insert((bx, bz), true);

      for y in 1..=height {
        let is_edge = bx == start_x
          || bx == start_x + width - 1
          || bz == start_z
          || bz == start_z + depth - 1
          || bx == courtyard_start_x - 1
          || bx == courtyard_start_x + courtyard_width
          || bz == courtyard_start_z - 1
          || bz == courtyard_start_z + courtyard_depth;
        let is_top = y == height;

        place_building_block(voxel_world, bx, y, bz, building_type, is_edge, is_top, rng);
      }
    }
  }
}

fn generate_cross_building(
  voxel_world: &mut VoxelWorld<MyWorld>,
  occupied: &mut std::collections::HashMap<(i32, i32), bool>,
  start_x: i32,
  start_z: i32,
  width: i32,
  depth: i32,
  height: i32,
  building_type: BuildingType,
  rng: &mut dyn FnMut() -> u32
) {
  let arm_width = width / 3;
  let arm_depth = depth / 3;

  // Horizontal bar
  let h_start_x = start_x;
  let h_start_z = start_z + depth / 2 - arm_depth / 2;

  // Vertical bar
  let v_start_x = start_x + width / 2 - arm_width / 2;
  let v_start_z = start_z;

  // Build horizontal bar
  for bx in h_start_x..(h_start_x + width) {
    for bz in h_start_z..(h_start_z + arm_depth) {
      occupied.insert((bx, bz), true);
      for y in 1..=height {
        let is_edge = bx == h_start_x
          || bx == h_start_x + width - 1
          || bz == h_start_z
          || bz == h_start_z + arm_depth - 1;
        let is_top = y == height;
        place_building_block(voxel_world, bx, y, bz, building_type, is_edge, is_top, rng);
      }
    }
  }

  // Build vertical bar
  for bx in v_start_x..(v_start_x + arm_width) {
    for bz in v_start_z..(v_start_z + depth) {
      if let std::collections::hash_map::Entry::Vacant(e) = occupied.entry((bx, bz)) {
        e.insert(true);
        for y in 1..=height {
          let is_edge = bx == v_start_x
            || bx == v_start_x + arm_width - 1
            || bz == v_start_z
            || bz == v_start_z + depth - 1;
          let is_top = y == height;
          place_building_block(voxel_world, bx, y, bz, building_type, is_edge, is_top, rng);
        }
      }
    }
  }
}

fn generate_t_building(
  voxel_world: &mut VoxelWorld<MyWorld>,
  occupied: &mut std::collections::HashMap<(i32, i32), bool>,
  start_x: i32,
  start_z: i32,
  width: i32,
  depth: i32,
  height: i32,
  building_type: BuildingType,
  rng: &mut dyn FnMut() -> u32
) {
  // Top horizontal bar
  let bar_depth = depth / 3;
  generate_simple_building(
    voxel_world,
    occupied,
    start_x,
    start_z,
    width,
    bar_depth,
    height,
    building_type,
    rng
  );

  // Vertical stem
  let stem_width = width / 3;
  let stem_start_x = start_x + width / 2 - stem_width / 2;
  let stem_start_z = start_z + bar_depth;
  let stem_depth = depth - bar_depth;
  generate_simple_building(
    voxel_world,
    occupied,
    stem_start_x,
    stem_start_z,
    stem_width,
    stem_depth,
    height,
    building_type,
    rng
  );
}

fn generate_multiple_towers_building(
  voxel_world: &mut VoxelWorld<MyWorld>,
  occupied: &mut std::collections::HashMap<(i32, i32), bool>,
  start_x: i32,
  start_z: i32,
  width: i32,
  depth: i32,
  height: i32,
  building_type: BuildingType,
  rng: &mut dyn FnMut() -> u32
) {
  // Base platform
  let base_height = height / 3;
  generate_simple_building(
    voxel_world,
    occupied,
    start_x,
    start_z,
    width,
    depth,
    base_height,
    building_type,
    rng
  );

  // Multiple towers on the base
  let tower_size = (width.min(depth)) / 3;
  let positions = [
    (start_x + 1, start_z + 1),
    (start_x + width - tower_size - 1, start_z + 1),
    (start_x + 1, start_z + depth - tower_size - 1),
    (start_x + width - tower_size - 1, start_z + depth - tower_size - 1)
  ];

  for (tx, tz) in positions.iter() {
    if *tx >= start_x
      && *tz >= start_z
      && *tx + tower_size <= start_x + width
      && *tz + tower_size <= start_z + depth
    {
      let tower_height = base_height + (height - base_height) * (3 + (rng() % 3) as i32) / 4;
      for bx in *tx..(*tx + tower_size) {
        for bz in *tz..(*tz + tower_size) {
          for y in (base_height + 1)..=tower_height {
            let is_edge = bx == *tx
              || bx == *tx + tower_size - 1
              || bz == *tz
              || bz == *tz + tower_size - 1;
            let is_top = y == tower_height;
            place_building_block(
              voxel_world,
              bx,
              y,
              bz,
              building_type,
              is_edge,
              is_top,
              rng
            );
          }
        }
      }
    }
  }
}

fn generate_curved_building(
  voxel_world: &mut VoxelWorld<MyWorld>,
  occupied: &mut std::collections::HashMap<(i32, i32), bool>,
  start_x: i32,
  start_z: i32,
  width: i32,
  depth: i32,
  height: i32,
  building_type: BuildingType,
  rng: &mut dyn FnMut() -> u32
) {
  // Create a curved building using an arc approximation
  let center_x = start_x + width / 2;
  let center_z = start_z + depth / 2;
  let radius = width.min(depth) as f32 / 2.0 - 1.0;

  for bx in start_x..(start_x + width) {
    for bz in start_z..(start_z + depth) {
      let dx = (bx - center_x) as f32;
      let dz = (bz - center_z) as f32;
      let distance = (dx * dx + dz * dz).sqrt();

      // Create a ring-shaped building (curved walls with hollow center)
      if distance >= radius - 2.0 && distance <= radius + 1.0 {
        occupied.insert((bx, bz), true);

        for y in 1..=height {
          let is_edge = distance >= radius + 0.5 || distance <= radius - 1.5;
          let is_top = y == height;
          place_building_block(voxel_world, bx, y, bz, building_type, is_edge, is_top, rng);
        }
      }
    }
  }
}

fn generate_ziggurat_building(
  voxel_world: &mut VoxelWorld<MyWorld>,
  occupied: &mut std::collections::HashMap<(i32, i32), bool>,
  start_x: i32,
  start_z: i32,
  width: i32,
  depth: i32,
  height: i32,
  building_type: BuildingType,
  rng: &mut dyn FnMut() -> u32
) {
  // More dramatic stepped pyramid with multiple levels
  let levels = 5.min(height / 3);
  let level_height = height / levels;

  for level in 0..levels {
    let shrink = level * 2;
    let level_width = (width - shrink).max(3);
    let level_depth = (depth - shrink).max(3);
    let level_start_x = start_x + shrink / 2;
    let level_start_z = start_z + shrink / 2;
    let level_bottom = level * level_height + 1;
    let level_top = (level + 1) * level_height;

    for bx in level_start_x..(level_start_x + level_width) {
      for bz in level_start_z..(level_start_z + level_depth) {
        if level == 0 {
          occupied.insert((bx, bz), true);
        }

        for y in level_bottom..=level_top {
          let is_edge = bx == level_start_x
            || bx == level_start_x + level_width - 1
            || bz == level_start_z
            || bz == level_start_z + level_depth - 1;
          let is_top = y == level_top;

          place_building_block(voxel_world, bx, y, bz, building_type, is_edge, is_top, rng);
        }
      }
    }
  }
}

fn generate_city(
  mut voxel_world: VoxelWorld<MyWorld>,
  mut commands: Commands,
  assets: Res<GameAssets>
) {
  use std::collections::HashMap;

  let mut noise_seed = 12345u32;
  let mut rng = || {
    noise_seed = noise_seed.wrapping_mul(1103515245).wrapping_add(12345);
    noise_seed
  };

  let mut occupied = HashMap::new();

  // First pass: generate sophisticated road network
  for x in -CITY_SIZE..CITY_SIZE {
    for z in -CITY_SIZE..CITY_SIZE {
      let road_width = get_road_width(x, z);
      let mut is_road = false;

      // Check if this position is within road width of any road line
      for offset in -(road_width / 2)..=(road_width / 2) {
        if is_main_road(x + offset, z) || is_main_road(x, z + offset) {
          is_road = true;
          break;
        }
      }

      if is_road {
        voxel_world.set_voxel(
          IVec3::new(x, 0, z),
          WorldVoxel::Solid(BlockType::Road.texture_coords() as u8)
        );
        occupied.insert((x, z), true);

        // Place trees at major intersections in park districts
        let district = get_district_type(x, z);
        if (x % 30 == 0 && z % 30 == 0) && district == DistrictType::Park {
          let distance_from_center = ((x * x + z * z) as f32).sqrt();
          if distance_from_center < 50.0 && (rng() % 3) == 0 {
            spawn_tree(&mut commands, &assets, Vec3::new(x as f32, 0.0, z as f32));
          }
        }
      } else {
        // Place sidewalks adjacent to roads
        let is_sidewalk = {
          let mut near_road = false;
          for dx in -2..=2 {
            for dz in -2..=2 {
              if dx == 0 && dz == 0 {
                continue;
              }
              let check_x = x + dx;
              let check_z = z + dz;
              if is_main_road(check_x, check_z) {
                near_road = true;
                break;
              }
            }
            if near_road {
              break;
            }
          }
          near_road
        };

        if is_sidewalk {
          voxel_world.set_voxel(
            IVec3::new(x, 0, z),
            WorldVoxel::Solid(BlockType::Sidewalk.texture_coords() as u8)
          );

          // Spawn NPCs on sidewalks frequently for testing
          if (rng() % 50) == 0 {
            let distance_from_center = ((x * x + z * z) as f32).sqrt();
            if distance_from_center < 200.0 && distance_from_center > 20.0 {
              println!("Spawning NPC at ({}, {})", x, z);
              spawn_npc(
                &mut commands,
                &assets,
                Vec3::new(x as f32, 0.0, z as f32),
                &mut rng
              );
            }
          }
        } else {
          voxel_world.set_voxel(
            IVec3::new(x, 0, z),
            WorldVoxel::Solid(BlockType::Grass.texture_coords() as u8)
          );
        }
      }
    }
  }

  // Second pass: simple building generation
  let mut buildings_placed = 0;

  // Place buildings on a much larger grid
  for x in (-250..250).step_by(10) {
    for z in (-250..250).step_by(10) {
      // Skip some randomly for variety
      if (rng() % 100) < 20 {
        continue;
      }

      let building_type = get_building_type_for_district(DistrictType::Downtown, &mut rng);
      let width = 3 + (rng() % 5) as i32;
      let depth = 3 + (rng() % 5) as i32;
      let height = 2 + (rng() % 4) as i32; // Much shorter: 2-5 blocks

      generate_complex_building(
        &mut voxel_world,
        &mut occupied,
        x,
        z,
        width,
        depth,
        height,
        building_type,
        &mut rng
      );
      buildings_placed += 1;
    }
  }
  println!("City generation complete: placed {} buildings", buildings_placed);
}

fn spawn_npc(
  c: &mut Commands,
  assets: &GameAssets,
  position: Vec3,
  _rng: &mut dyn FnMut() -> u32
) {
  let world_pos = Vec3::new(position.x * BLOCK_SIZE, 2.0, position.z * BLOCK_SIZE); // Raised to 2.0 for better visibility

  // Simple single entity - no physics, just visual, same size as player
  c.spawn((
    Transform::from_translation(world_pos),
    Npc { wants_pizza: true, satisfied: false },
    BillboardTexture(assets.woman1_texture.clone()),
    BillboardMesh(assets.npc_mesh.clone()),
    BillboardLockAxis { y_axis: false, rotation: false }
  ));
}

fn spawn_tree(c: &mut Commands, assets: &GameAssets, position: Vec3) {
  let world_pos = Vec3::new(position.x * BLOCK_SIZE, 1.0, position.z * BLOCK_SIZE);

  // Simple single entity - no physics
  c.spawn((
    Transform::from_translation(world_pos),
    Tree,
    BillboardTexture(assets.tree_texture.clone()),
    BillboardMesh(assets.tree_mesh.clone()),
    BillboardLockAxis::from_lock_y(true)
  ));
}

fn spawn_player(mut c: Commands, assets: Res<GameAssets>) {
  let _player_entity = c
    .spawn((
      Transform::from_translation(Vec3::new(5.0, 15.0, 5.0)),
      RigidBody::Dynamic,
      Collider::sphere(0.5), // Sphere collider for smoother physics
      ColliderDensity(1.0),
      Friction::new(0.3),
      Restitution::new(0.0),
      ExternalForce::default(), // For force-based movement
      Player,
      BillboardTexture(assets.player_texture.clone()),
      BillboardMesh(assets.player_mesh.clone()),
      BillboardLockAxis { y_axis: false, rotation: false }
    ))
    .id();
}

fn player_move_and_jump(
  mut player_query: Query<
    (&mut ExternalForce, &mut LinearVelocity, &Transform, Entity),
    With<Player>
  >,
  camera_query: Query<&Transform, With<Camera3d>>,
  keyboard: Res<ButtonInput<KeyCode>>,
  spatial_query: SpatialQuery
) {
  // Grab the single player tuple and the camera transform
  if let (Ok((mut ext_force, mut vel, player_tf, player_ent)), Ok(camera_tf)) =
    (player_query.single_mut(), camera_query.single())
  {
    /* ---------- 1. movement ---------- */

    // ExternalForce should only last one frame
    ext_force.persistent = false;

    // Ground check (sphere-cast down a little)
    let ray_origin = player_tf.translation;
    let ray_direction = Dir3::NEG_Y;
    let max_distance = 0.7;
    let filter = SpatialQueryFilter::default().with_excluded_entities([player_ent]);

    let is_grounded = spatial_query
      .cast_ray(ray_origin, ray_direction, max_distance, true, &filter)
      .is_some(); // ← remove the old “|| true” debug hack

    // Camera-relative axes, flattened to X-Z
    let cam_fwd = camera_tf.forward().normalize();
    let cam_right = camera_tf.right().normalize();
    let forward = Vec3::new(cam_fwd.x, 0.0, cam_fwd.z).normalize();
    let right = Vec3::new(cam_right.x, 0.0, cam_right.z).normalize();

    // Direction from WASD / arrows
    let input_dir = [
      (KeyCode::KeyW, forward),
      (KeyCode::ArrowUp, forward),
      (KeyCode::KeyS, -forward),
      (KeyCode::ArrowDown, -forward),
      (KeyCode::KeyA, -right),
      (KeyCode::ArrowLeft, -right),
      (KeyCode::KeyD, right),
      (KeyCode::ArrowRight, right)
    ]
    .into_iter()
    .filter_map(|(k, v)| keyboard.pressed(k).then_some(v))
    .sum::<Vec3>()
    .normalize_or_zero();

    // Tunables
    let max_speed = 8.0;
    let max_force = 200.0;
    let friction_force = 150.0;

    // Current planar velocity
    let horiz_vel = Vec3::new(vel.x, 0.0, vel.z);
    let speed = horiz_vel.length();

    // Choose a single force for this frame
    let force = if is_grounded {
      if input_dir.length() > 0.1 {
        // Driving + friction while a key is held
        let speed_ratio = (speed / max_speed).min(1.0);
        // Optional extra friction (commented out in original)
        /* let friction    = if speed > 0.1 {
            -horiz_vel.normalize() * friction_force * speed_ratio
        } else { Vec3::ZERO }; */
        input_dir * max_force * (1.0 - speed_ratio)
      } else if speed > 0.1 {
        // No input: braking friction
        -horiz_vel.normalize() * friction_force
      } else {
        Vec3::ZERO
      }
    } else {
      // Airborne: no ground forces
      Vec3::ZERO
    };

    ext_force.apply_force(force);

    /* ---------- 2. jump ---------- */

    if is_grounded && keyboard.just_pressed(KeyCode::Space) {
      vel.y = 12.0; // instant upward kick
    }
  }
}

fn throw_pizza(
  mut commands: Commands,
  assets: Res<GameAssets>,
  mut stats: ResMut<PlayerStats>,
  player_query: Query<&Transform, With<Player>>,
  keyboard: Res<ButtonInput<KeyCode>>,
  camera_query: Query<&Transform, (With<Camera3d>, Without<Player>)>
) {
  if keyboard.just_pressed(KeyCode::KeyE)
    && stats.pizzas_remaining > 0
    && let Ok(player_transform) = player_query.single()
    && let Ok(camera_transform) = camera_query.single()
  {
    let throw_direction =
      (*camera_transform.forward() + Vec3::new(0.0, 0.3, 0.0)).normalize();
    let spawn_pos = player_transform.translation + Vec3::new(0.0, 1.0, 0.0);

    // Simple single entity using same approach as NPCs - just visual billboard with physics
    println!(
      "Throwing pizza at position: {:?} with direction: {:?}",
      spawn_pos, throw_direction
    );
    commands.spawn((
      Transform::from_translation(spawn_pos),
      RigidBody::Dynamic,
      Collider::sphere(0.2),
      LinearVelocity(throw_direction * 15.0),
      Restitution::new(0.8),
      Friction::new(0.0),
      LinearDamping(0.0),
      AngularDamping(1.0),
      Pizza,
      PizzaTimer::new(),
      BillboardTexture(assets.pizza_texture.clone()),
      BillboardMesh(assets.pizza_mesh.clone()),
      BillboardLockAxis { y_axis: false, rotation: false }
    ));

    stats.pizzas_remaining -= 1;
  }
}

fn pizza_physics(
  pizza_query: Query<(Entity, &Transform), With<Pizza>>,
  mut commands: Commands,
  mut stats: ResMut<PlayerStats>
) {
  for (entity, transform) in pizza_query.iter() {
    // Despawn if falls too far
    if transform.translation.y < -10.0 {
      commands.entity(entity).despawn();
    }

    // Successful delivery if pizza travels far from origin
    if transform.translation.distance(Vec3::ZERO) > 100.0 {
      commands.entity(entity).despawn();
      stats.pizzas_delivered += 1;
    }
  }
}

fn pizza_delivery_system(
  pizza_query: Query<(Entity, &Transform), With<Pizza>>,
  mut npc_query: Query<(&mut Npc, &Transform)>,
  mut commands: Commands,
  mut stats: ResMut<PlayerStats>
) {
  for (pizza_entity, pizza_transform) in pizza_query.iter() {
    for (mut npc, npc_transform) in npc_query.iter_mut() {
      if npc.wants_pizza && !npc.satisfied {
        let distance = pizza_transform.translation.distance(npc_transform.translation);

        // Pizza must be close to NPC (within 2 units) to be delivered
        if distance < 2.0 {
          npc.satisfied = true;
          npc.wants_pizza = false;

          println!(
            "Pizza delivered to NPC at distance {:.2}! They are now satisfied.",
            distance
          );
          commands.entity(pizza_entity).despawn();
          stats.pizzas_delivered += 1;
          return; // Only deliver one pizza per frame
        }
      }
    }
  }
}

fn pizza_timer_system(
  mut pizza_query: Query<(Entity, &mut PizzaTimer), With<Pizza>>,
  mut commands: Commands,
  time: Res<Time>
) {
  for (entity, mut pizza_timer) in pizza_query.iter_mut() {
    pizza_timer.timer.tick(time.delta());

    // Despawn pizza after timer expires if not delivered
    if pizza_timer.timer.finished() {
      commands.entity(entity).despawn();
    }
  }
}

// sprite_follow_system removed - entities are now self-contained

fn camera_follow_player(
  player_query: Query<&Transform, With<Player>>,
  mut camera_query: Query<&mut CameraController>
) {
  if let Ok(player_transform) = player_query.single()
    && let Ok(mut controller) = camera_query.single_mut()
  {
    controller.target = player_transform.translation + Vec3::new(0.0, 2.0, 0.0);
  }
}

fn camera_controller(
  mut camera_query: Query<(&mut Transform, &mut CameraController), With<Camera3d>>,
  keyboard: Res<ButtonInput<KeyCode>>,
  time: Res<Time>
) {
  if let Ok((mut transform, mut controller)) = camera_query.single_mut() {
    if keyboard.pressed(KeyCode::KeyQ) {
      controller.yaw -= 2.0 * time.delta_secs();
    }
    if keyboard.pressed(KeyCode::KeyX) {
      controller.yaw += 2.0 * time.delta_secs();
    }
    if keyboard.pressed(KeyCode::KeyZ) {
      controller.pitch = (controller.pitch - 1.0 * time.delta_secs()).clamp(-1.5, 0.0);
    }
    if keyboard.pressed(KeyCode::KeyC) {
      controller.pitch = (controller.pitch + 1.0 * time.delta_secs()).clamp(-1.5, 0.0);
    }
    // Zoom controls moved to mouse wheel in mouse_look function

    let rotation = Quat::from_euler(EulerRot::YXZ, controller.yaw, controller.pitch, 0.0);
    let offset = rotation * Vec3::new(0.0, 0.0, controller.distance);

    transform.translation = controller.target + offset;
    transform.look_at(controller.target, Vec3::Y);
  }
}

fn mouse_look(
  mut camera_query: Query<&mut CameraController>,
  mut mouse_motion: EventReader<MouseMotion>,
  mut mouse_wheel: EventReader<MouseWheel>,
  mouse_button: Res<ButtonInput<MouseButton>>
) {
  if let Ok(mut controller) = camera_query.single_mut() {
    if mouse_button.pressed(MouseButton::Right) {
      for event in mouse_motion.read() {
        controller.yaw -= event.delta.x * 0.003;
        controller.pitch = (controller.pitch - event.delta.y * 0.003).clamp(-1.5, 0.0);
      }
    }

    // Handle mouse wheel zoom
    for event in mouse_wheel.read() {
      controller.distance = (controller.distance - event.y * 2.0).clamp(5.0, 50.0);
    }
  }
}

fn add_colliders_to_chunks(
  mut commands: Commands,
  mesh_query: Query<(Entity, &Mesh3d), Without<Collider>>,
  meshes: Res<Assets<Mesh>>
) {
  for (entity, mesh3d) in mesh_query.iter() {
    if let Some(mesh) = meshes.get(&mesh3d.0)
      && let Some(positions) = mesh.attribute(Mesh::ATTRIBUTE_POSITION)
      && !positions.is_empty()
      && let Some(collider) = Collider::trimesh_from_mesh(mesh)
    {
      commands.entity(entity).insert((
        RigidBody::Static,
        collider,
        Friction::new(0.0),    // No friction on ground
        Restitution::new(0.0)  // No bouncing for ground
      ));
    }
  }
}

fn update_ui(
  mut commands: Commands,
  stats: Res<PlayerStats>,
  ui_root_query: Query<Entity, With<Node>>
) {
  for entity in ui_root_query.iter() {
    commands.entity(entity).despawn();
  }

  // Top-left UI panel
  commands
    .spawn((
      Node {
        position_type: PositionType::Absolute,
        top: Val::Px(20.0),
        left: Val::Px(20.0),
        flex_direction: FlexDirection::Column,
        row_gap: Val::Px(10.0),
        padding: UiRect::all(Val::Px(15.0)),
        ..default()
      },
      BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.7)) // Semi-transparent background
    ))
    .with_children(|parent| {
      parent.spawn((
        Text::new(format!("Pizzas Remaining: {}", stats.pizzas_remaining)),
        TextFont { font_size: 24.0, ..default() },
        TextColor(Color::WHITE)
      ));

      parent.spawn((
        Text::new(format!("Pizzas Delivered: {}", stats.pizzas_delivered)),
        TextFont { font_size: 24.0, ..default() },
        TextColor(Color::WHITE)
      ));
    });

  // Bottom controls text
  commands.spawn((
    Node {
      position_type: PositionType::Absolute,
      bottom: Val::Px(20.0),
      left: Val::Px(20.0),
      right: Val::Px(20.0),
      padding: UiRect::all(Val::Px(15.0)),
      justify_content: JustifyContent::Center,
      ..default()
    },
    BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.7)),
  )).with_children(|parent| {
    parent.spawn((
      Text::new("Controls: WASD = Move, Space = Jump, E = Throw Pizza, Right Click + Mouse = Look, QX = Rotate, ZC = Pitch, Mouse Wheel = Zoom"),
      TextFont {
        font_size: 16.0,
        ..default()
      },
      TextColor(Color::WHITE),
      Node {
        max_width: Val::Percent(100.0),
        ..default()
      },
    ));
  });
}
