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

impl BlockType {
  const TEXTURE_SIZE: u32 = 16;
  const SIDE_LENGTH_PIXELS: u32 = 10;

  fn get_texture_atlas_index(self) -> usize { self.texture_coords() as usize }
}

const CITY_SIZE: i32 = 100;
const BLOCK_SIZE: f32 = 1.0;
const BUILDING_HEIGHT_MIN: i32 = 15;
const BUILDING_HEIGHT_MAX: i32 = 25;

#[derive(Component)]
struct Player;

#[derive(Component)]
struct Pizza;

#[derive(Component)]
struct PizzaTimer {
  timer: Timer
}

impl PizzaTimer {
  fn new() -> Self { Self { timer: Timer::from_seconds(3.0, TimerMode::Once) } }
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
  wants_pizza: bool
}

// Component to link sprites to their physics bodies
#[derive(Component)]
struct SpriteFollower {
  target_entity: Entity
}

#[derive(Resource, Clone, Default)]
struct MyWorld;

impl VoxelWorldConfig for MyWorld {
  type MaterialIndex = u8;
  type ChunkUserBundle = (RigidBody, Collider, Friction, Restitution);

  fn spawning_distance(&self) -> u32 { 50 }

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
  quad_mesh: Handle<Mesh>
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
    .insert_resource(PlayerStats { pizzas_delivered: 0, pizzas_remaining: 10 })
    .add_systems(
      Startup,
      (setup_assets, setup_camera, setup_voxel_world, generate_city, spawn_player).chain()
    )
    .add_systems(
      Update,
      (
        player_movement,
        player_jump,
        throw_pizza,
        pizza_physics,
        pizza_timer_system,
        pizza_delivery_system,
        sprite_follow_system,
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
  // Voxel materials are handled by the voxel world system

  commands.insert_resource(GameAssets {
    player_texture,
    pizza_texture,
    tree_texture,
    woman1_texture,
    ducky_texture,
    blocks_layout,
    block_material,
    quad_mesh
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

fn generate_city(
  mut voxel_world: VoxelWorld<MyWorld>,
  mut commands: Commands,
  assets: Res<GameAssets>,
  mut meshes: ResMut<Assets<Mesh>>
) {
  use std::collections::HashMap;

  let mut noise_seed = 12345u32;
  let mut rng = || {
    noise_seed = noise_seed.wrapping_mul(1103515245).wrapping_add(12345);
    noise_seed
  };

  let mut occupied = HashMap::new();
  let block_size = 20; // Size of each city block (building + space)

  // First pass: generate roads and sidewalks
  for x in -CITY_SIZE..CITY_SIZE {
    for z in -CITY_SIZE..CITY_SIZE {
      let is_road = (x % block_size == 0) || (z % block_size == 0);

      if is_road {
        voxel_world.set_voxel(
          IVec3::new(x, 0, z),
          WorldVoxel::Solid(BlockType::Road.texture_coords() as u8)
        );

        if x % block_size == 0 && z % block_size == 0 {
          let distance_from_center = ((x * x + z * z) as f32).sqrt();
          if distance_from_center < 30.0 {
            spawn_tree(
              &mut commands,
              &assets,
              &mut meshes,
              Vec3::new(x as f32, 0.0, z as f32)
            );
          }
        }
      } else {
        voxel_world.set_voxel(
          IVec3::new(x, 0, z),
          WorldVoxel::Solid(BlockType::Sidewalk.texture_coords() as u8)
        );

        // Spawn NPCs on sidewalks occasionally
        if (rng() % 100) == 0 {
          let distance_from_center = ((x * x + z * z) as f32).sqrt();
          if distance_from_center < 80.0 && distance_from_center > 10.0 {
            spawn_npc(
              &mut commands,
              &assets,
              &mut meshes,
              Vec3::new(x as f32, 0.0, z as f32),
              &mut rng
            );
          }
        }
      }
    }
  }

  // Second pass: generate buildings in plots
  for block_x in (-CITY_SIZE / block_size)..(CITY_SIZE / block_size) {
    for block_z in (-CITY_SIZE / block_size)..(CITY_SIZE / block_size) {
      let center_x = block_x * block_size + block_size / 2;
      let center_z = block_z * block_size + block_size / 2;
      let distance_from_center = ((center_x * center_x + center_z * center_z) as f32).sqrt();

      if distance_from_center < 35.0 && (rng() % 100) < 60 {
        // Generate a building of random size within the plot
        let building_width = 3 + (rng() % 8) as i32; // 3-10 blocks wide
        let building_depth = 3 + (rng() % 8) as i32; // 3-10 blocks deep
        let building_height = BUILDING_HEIGHT_MIN
          + ((rng() % (BUILDING_HEIGHT_MAX - BUILDING_HEIGHT_MIN) as u32) as i32);

        // Center the building in the plot with some spacing
        let start_x = center_x - building_width / 2;
        let start_z = center_z - building_depth / 2;

        // Ensure building doesn't go into roads (leave 2 block buffer)
        let min_x = block_x * block_size + 2;
        let max_x = (block_x + 1) * block_size - 2;
        let min_z = block_z * block_size + 2;
        let max_z = (block_z + 1) * block_size - 2;

        for bx in start_x.max(min_x)..(start_x + building_width).min(max_x) {
          for bz in start_z.max(min_z)..(start_z + building_depth).min(max_z) {
            if let std::collections::hash_map::Entry::Vacant(e) = occupied.entry((bx, bz)) {
              e.insert(true);

              for y in 1..=building_height {
                let block_type = if y == building_height {
                  BlockType::Roof
                } else if y == 1 && (bx == start_x || bz == start_z) && (rng() % 6) == 0 {
                  BlockType::Door
                } else if (rng() % 4) == 0 {
                  BlockType::Window
                } else {
                  BlockType::Building
                };
                voxel_world.set_voxel(
                  IVec3::new(bx, y, bz),
                  WorldVoxel::Solid(block_type.texture_coords() as u8)
                );
              }
            }
          }
        }
      }
    }
  }
}

fn spawn_npc(
  c: &mut Commands,
  assets: &GameAssets,
  meshes: &mut ResMut<Assets<Mesh>>,
  position: Vec3,
  rng: &mut dyn FnMut() -> u32
) {
  let world_pos = Vec3::new(position.x * BLOCK_SIZE, 0.5, position.z * BLOCK_SIZE);

  // Create physics body
  let npc_entity = c
    .spawn((
      Transform::from_translation(world_pos),
      RigidBody::Static,
      Collider::capsule(0.3, 0.8),
      Npc { wants_pizza: (rng() % 3) == 0 } // 1/3 chance of wanting pizza
    ))
    .id();

  // Choose random NPC texture
  let texture = if (rng() % 2) == 0 {
    assets.woman1_texture.clone()
  } else {
    assets.ducky_texture.clone()
  };

  // Create sprite follower entity
  let sprite_follower = c
    .spawn((Transform::from_translation(world_pos), SpriteFollower {
      target_entity: npc_entity
    }))
    .id();

  // Create billboard as child of sprite follower
  c.spawn((
    BillboardTexture(texture),
    BillboardMesh(meshes.add(Rectangle::new(1.65, 2.75))), // 10% taller
    Transform::from_translation(Vec3::new(0.0, 1.375, 0.0)),
    BillboardLockAxis::from_lock_y(true),
    ChildOf(sprite_follower)
  ));
}

fn spawn_tree(
  c: &mut Commands,
  assets: &GameAssets,
  meshes: &mut ResMut<Assets<Mesh>>,
  position: Vec3
) {
  let world_pos = Vec3::new(position.x * BLOCK_SIZE, 1.0, position.z * BLOCK_SIZE);

  // Create physics body
  let tree_entity = c
    .spawn((
      Transform::from_translation(world_pos),
      RigidBody::Static,
      Collider::cuboid(0.3, 1.0, 0.3),
      Tree
    ))
    .id();

  // Create sprite follower entity
  let sprite_follower = c
    .spawn((Transform::from_translation(world_pos), SpriteFollower {
      target_entity: tree_entity
    }))
    .id();

  // Create billboard as child of sprite follower - position it so bottom of tree touches ground
  c.spawn((
    BillboardTexture(assets.tree_texture.clone()),
    BillboardMesh(meshes.add(Rectangle::new(1.5, 2.0))),
    Transform::from_translation(Vec3::new(0.0, 1.0, 0.0)), // Half the tree height above ground
    BillboardLockAxis::from_lock_y(true), // Lock Y axis for vertical billboards
    ChildOf(sprite_follower)
  ));
}

fn spawn_player(mut c: Commands, assets: Res<GameAssets>, mut meshes: ResMut<Assets<Mesh>>) {
  // Spawn at (5, 3, 5) to avoid trees at intersections and be closer to ground
  let player_entity = c
    .spawn((
      Transform::from_translation(Vec3::new(5.0, 70.0, 5.0)),
      RigidBody::Dynamic,
      Collider::sphere(0.5), // Sphere collider for smoother physics
      ColliderDensity(1.0),
      // ColliderMassProperties::
      // LockedAxes::ROTATION_LOCKED,
      // LinearDamping(0.1), // Tiny bit of damping to prevent instability
      // AngularDamping(10.0),
      Friction::new(0.0),    // No physics friction - we implement our own
      Restitution::new(0.0), // No bounciness for player
      ExternalForce::default(), // For force-based movement
      Player,
      BillboardTexture(assets.player_texture.clone()),
      BillboardMesh(meshes.add(Rectangle::new(1.0, 2.0))),
      BillboardLockAxis::from_lock_y(true) // Lock Y axis for vertical billboards
    ))
    .id();

  // Create sprite follower entity for billboard
  // let sprite_follower = c
  //   .spawn((Transform::from_translation(Vec3::new(5.0, 3.0, 5.0)), SpriteFollower {
  //     target_entity: player_entity
  //   }))
  //   .id();

  // Create billboard as child of sprite follower
  // c.spawn((
  //   BillboardTexture(assets.player_texture.clone()),
  //   BillboardMesh(meshes.add(Rectangle::new(1.0, 2.0))),
  //   Transform::from_translation(Vec3::new(0.0, 1.0, 0.0)), // Center the sprite vertically
  //   BillboardLockAxis::from_lock_y(true), // Lock Y axis for vertical billboards
  //   ChildOf(sprite_follower)
  // ));
}

fn player_movement(
  mut player_query: Query<(&mut ExternalForce, &LinearVelocity, Entity), With<Player>>,
  player_transform_query: Query<&Transform, With<Player>>,
  camera_query: Query<&Transform, With<Camera3d>>,
  keyboard: Res<ButtonInput<KeyCode>>,
  spatial_query: SpatialQuery
) {
  if let (
    Ok((mut external_force, velocity, player_entity)),
    Ok(player_transform),
    Ok(camera_transform)
  ) = (player_query.single_mut(), player_transform_query.single(), camera_query.single())
  {
    // not persistent
    external_force.persistent = false;

    // Check if player is on ground
    // Player sphere collider has radius 0.5, so cast from bottom of sphere down
    let ray_origin = player_transform.translation;
    let ray_direction = Dir3::NEG_Y;
    let max_distance = 0.7; // Short distance to detect very close ground
    let filter = SpatialQueryFilter::default().with_excluded_entities([player_entity]);
    let raycast_result =
      spatial_query.cast_ray(ray_origin, ray_direction, max_distance, true, &filter);
    let is_grounded = raycast_result.is_some() || true;

    // Get input direction
    let camera_forward = camera_transform.forward().normalize();
    let camera_right = camera_transform.right().normalize();
    let forward = Vec3::new(camera_forward.x, 0.0, camera_forward.z).normalize();
    let right = Vec3::new(camera_right.x, 0.0, camera_right.z).normalize();


    let direction = [
      // (key, contribution)
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
    .filter_map(|(key, v)| keyboard.pressed(key).then_some(v)) // keep the vector only if the key is down
    .sum::<Vec3>() // Vec3 implements `Sum`
    .normalize_or_zero(); // guard against “no input”

    // Single force calculation per frame
    let max_speed = 8.0; // Reduced from 12.0
    let max_force = 200.0; // Reduced from 400.0  
    let friction_force = 150.0; // Reduced from 300.0

    // Current horizontal velocity
    let current_horizontal = Vec3::new(velocity.x, 0.0, velocity.z);
    let current_speed = current_horizontal.length();

    // Debug control loss - log when velocity gets too high or when no input but still moving
    // static mut DEBUG_COUNTER: u32 = 0;
    // unsafe {
    //   DEBUG_COUNTER += 1;
    //   let has_input = direction.length() > 0.1;

    //   if DEBUG_COUNTER % 60 == 0
    //     && (current_speed > 6.0 || (!has_input && current_speed > 2.0))
    //   {
    //     println!(
    //       "Control issue - pos: {:.2?}, grounded: {}, velocity: {:.2?}, input: {}, speed: {:.2}",
    //       player_transform.translation, is_grounded, velocity.0, has_input, current_speed
    //     );
    //     if let Some(hit) = raycast_result {
    //       println!("  Ground hit at distance: {:.3}", hit.distance);
    //     }
    //   }
    // }

    // Calculate single force for this frame
    let final_force = if is_grounded {
      if direction.length() > 0.1 {
        // Moving: combine input force and friction
        let speed_ratio = (current_speed / max_speed).min(1.0);
        let input_force = direction * max_force * (1.0 - speed_ratio); // Scales to zero at max speed

        // Custom friction: oppose current movement
        let friction = if current_speed > 0.1 {
          -current_horizontal.normalize() * friction_force * speed_ratio
        } else {
          Vec3::ZERO
        };

        input_force + friction
      } else {
        // Not moving: only friction to stop
        if current_speed > 0.1 {
          -current_horizontal.normalize() * friction_force
        } else {
          Vec3::ZERO
        }
      }
    } else {
      // In air: no forces applied
      Vec3::ZERO
    };

    // Apply the calculated force (forces were cleared at frame start)
    external_force.apply_force(final_force);
  }
}

fn player_jump(
  mut player_query: Query<(&mut LinearVelocity, Entity), With<Player>>,
  keyboard: Res<ButtonInput<KeyCode>>,
  spatial_query: SpatialQuery,
  player_transform_query: Query<&Transform, With<Player>>
) {
  if keyboard.just_pressed(KeyCode::Space)
    && let Ok((mut velocity, player_entity)) = player_query.single_mut()
    && let Ok(transform) = player_transform_query.single()
  {
    // Cast ray from slightly above the player's feet to detect ground
    let ray_origin = transform.translation + Vec3::new(0.0, -0.8, 0.0);
    let ray_direction = Dir3::NEG_Y;
    let max_distance = 0.3; // Shorter distance for more precise ground detection

    // Exclude the player entity from the raycast
    let filter = SpatialQueryFilter::default().with_excluded_entities([player_entity]);

    if let Some(_hit) =
      spatial_query.cast_ray(ray_origin, ray_direction, max_distance, true, &filter)
      && velocity.y.abs() < 2.0
    {
      // Direct velocity change for instant jump
      velocity.y = 12.0;
    }
  }
}

fn throw_pizza(
  mut commands: Commands,
  assets: Res<GameAssets>,
  mut stats: ResMut<PlayerStats>,
  player_query: Query<&Transform, With<Player>>,
  keyboard: Res<ButtonInput<KeyCode>>,
  camera_query: Query<&Transform, (With<Camera3d>, Without<Player>)>,
  mut meshes: ResMut<Assets<Mesh>>
) {
  if keyboard.just_pressed(KeyCode::KeyE)
    && stats.pizzas_remaining > 0
    && let Ok(player_transform) = player_query.single()
    && let Ok(camera_transform) = camera_query.single()
  {
    let throw_direction =
      (*camera_transform.forward() + Vec3::new(0.0, 0.3, 0.0)).normalize();
    let spawn_pos = player_transform.translation + Vec3::new(0.0, 1.0, 0.0);

    // Create physics body with bouncy physics
    let pizza_entity = commands
      .spawn((
        Transform::from_translation(spawn_pos),
        RigidBody::Dynamic,
        Collider::sphere(0.2),
        LinearVelocity(throw_direction * 15.0),
        Restitution::new(0.8), // High bounciness
        Friction::new(0.0),
        LinearDamping(0.0),
        AngularDamping(1.0), // Moderate spin damping
        Pizza,
        PizzaTimer::new()
      ))
      .id();

    // Create sprite follower entity
    let sprite_follower = commands
      .spawn((Transform::from_translation(spawn_pos), SpriteFollower {
        target_entity: pizza_entity
      }))
      .id();

    // Create billboard as child of sprite follower
    commands.spawn((
      BillboardTexture(assets.pizza_texture.clone()),
      BillboardMesh(meshes.add(Rectangle::new(1.0, 1.0))),
      Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
      BillboardLockAxis::from_lock_y(true), // Lock Y axis for vertical billboards
      ChildOf(sprite_follower)
    ));

    stats.pizzas_remaining -= 1;
  }
}

fn pizza_physics(
  pizza_query: Query<(Entity, &Transform), With<Pizza>>,
  sprite_follower_query: Query<(Entity, &SpriteFollower), Without<Pizza>>,
  mut commands: Commands,
  mut stats: ResMut<PlayerStats>
) {
  for (entity, transform) in pizza_query.iter() {
    let mut should_despawn = false;
    let mut delivered = false;

    // Despawn if falls too far
    if transform.translation.y < -10.0 {
      should_despawn = true;
    }

    // Successful delivery if pizza travels far from origin
    if transform.translation.distance(Vec3::ZERO) > 100.0 {
      should_despawn = true;
      delivered = true;
    }

    if should_despawn {
      // Find and despawn associated sprite followers
      for (follower_entity, sprite_follower) in sprite_follower_query.iter() {
        if sprite_follower.target_entity == entity {
          commands.entity(follower_entity).despawn();
        }
      }

      commands.entity(entity).despawn();

      if delivered {
        stats.pizzas_delivered += 1;
      }
    }
  }
}

fn pizza_delivery_system(
  pizza_query: Query<(Entity, &Transform), With<Pizza>>,
  mut npc_query: Query<(Entity, &Transform, &mut Npc)>,
  sprite_follower_query: Query<(Entity, &SpriteFollower), Without<Pizza>>,
  mut commands: Commands,
  mut stats: ResMut<PlayerStats>
) {
  for (pizza_entity, pizza_transform) in pizza_query.iter() {
    for (_npc_entity, npc_transform, mut npc) in npc_query.iter_mut() {
      // Check if NPC wants pizza and pizza is close enough
      if npc.wants_pizza {
        let distance = pizza_transform.translation.distance(npc_transform.translation);
        if distance < 3.0 {
          // Increased delivery range
          // Mark NPC as satisfied
          npc.wants_pizza = false;

          // Remove pizza and its sprite
          for (follower_entity, sprite_follower) in sprite_follower_query.iter() {
            if sprite_follower.target_entity == pizza_entity {
              commands.entity(follower_entity).despawn();
            }
          }
          commands.entity(pizza_entity).despawn();

          // Increment delivery counter
          stats.pizzas_delivered += 1;

          break; // Exit NPC loop for this pizza
        }
      }
    }
  }
}

fn pizza_timer_system(
  mut pizza_query: Query<(Entity, &mut PizzaTimer), With<Pizza>>,
  sprite_follower_query: Query<(Entity, &SpriteFollower), Without<Pizza>>,
  mut commands: Commands,
  time: Res<Time>
) {
  for (entity, mut pizza_timer) in pizza_query.iter_mut() {
    pizza_timer.timer.tick(time.delta());

    // Despawn pizza after 3 seconds if not delivered
    if pizza_timer.timer.finished() {
      // Find and despawn associated sprite followers
      for (follower_entity, sprite_follower) in sprite_follower_query.iter() {
        if sprite_follower.target_entity == entity {
          commands.entity(follower_entity).despawn();
        }
      }

      commands.entity(entity).despawn();
    }
  }
}

fn sprite_follow_system(
  mut sprite_query: Query<(&mut Transform, &SpriteFollower)>,
  target_query: Query<
    &Transform,
    (Without<SpriteFollower>, Or<(With<Pizza>, With<Tree>, With<Npc>, With<Player>)>)
  >
) {
  for (mut sprite_transform, follower) in sprite_query.iter_mut() {
    if let Ok(target_transform) = target_query.get(follower.target_entity) {
      // Position sprite at the same location as physics body
      sprite_transform.translation = target_transform.translation;
    }
  }
}

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
