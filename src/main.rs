#![feature(let_chains)]
#![allow(dead_code)]

use bevy::asset::AssetMetaCheck;
use bevy::prelude::*;
// use bevy::math::Affine2; // Not needed with voxel world
use bevy::input::mouse::MouseMotion;
use avian3d::prelude::*;
// use avian3d::collision::AsyncCollider; // Not available in this version
use bevy_embedded_assets::EmbeddedAssetPlugin;
use enum_assoc::Assoc;
use bevy_voxel_world::prelude::*;

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
    GrayBricks,
}

impl BlockType {
    const TEXTURE_SIZE: u32 = 16;
    const SIDE_LENGTH_PIXELS: u32 = 10;
    
    fn get_texture_atlas_index(self) -> usize {
        self.texture_coords() as usize
    }
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
struct CityBlock {
    block_type: BlockType,
    position: IVec3,
}

#[derive(Component)]
struct Billboard;

#[derive(Component)]
struct Tree;

#[derive(Component)]
enum FacingMode {
    PositionIgnoreY,
    Position,
    Direction,
}

#[derive(Resource, Clone, Default)]
struct MyWorld;

impl VoxelWorldConfig for MyWorld {
    type MaterialIndex = u8;
    type ChunkUserBundle = (RigidBody, Collider);

    fn spawning_distance(&self) -> u32 {
        50
    }

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
    blocks_layout: Handle<TextureAtlasLayout>,
    block_material: Handle<StandardMaterial>,
    player_material: Handle<StandardMaterial>,
    pizza_material: Handle<StandardMaterial>,
    tree_material: Handle<StandardMaterial>,
    quad_mesh: Handle<Mesh>,
}

#[derive(Resource)]
struct PlayerStats {
    pizzas_delivered: u32,
    pizzas_remaining: u32,
}

#[derive(Component)]
struct CameraController {
    target: Vec3,
    distance: f32,
    pitch: f32,
    yaw: f32,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(AssetPlugin {
            meta_check: AssetMetaCheck::Never,
            ..default()
        }).set(ImagePlugin::default_nearest()))
        .add_plugins(EmbeddedAssetPlugin::default())
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(VoxelWorldPlugin::with_config(MyWorld))
        .insert_resource(PlayerStats {
            pizzas_delivered: 0,
            pizzas_remaining: 10,
        })
        .add_systems(Startup, (setup_assets, setup_camera, setup_voxel_world, generate_city, spawn_player).chain())
        .add_systems(Update, (
            player_movement,
            player_jump,
            throw_pizza,
            pizza_physics,
            camera_follow_player,
            face_camera_system,
            camera_controller,
            mouse_look,
        ))
        .add_systems(Update, (
            add_colliders_to_chunks,
            update_ui,
        ))
        .run();
}

fn setup_assets(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut texture_atlas_layouts: ResMut<Assets<TextureAtlasLayout>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    // Don't load blocks_texture here since voxel world handles it
    let player_texture = asset_server.load("guy.png");
    let pizza_texture = asset_server.load("pizzabox.png");
    let tree_texture = asset_server.load("tree.png");
    
    // Remove texture atlas layout since voxel world handles textures
    let blocks_layout = texture_atlas_layouts.add(TextureAtlasLayout::new_empty(UVec2::splat(10)));
    
    // Remove block_material since voxel world handles its own materials
    let block_material = materials.add(StandardMaterial::default());
    
    let player_material = materials.add(StandardMaterial {
        base_color_texture: Some(player_texture.clone()),
        alpha_mode: AlphaMode::Blend,
        unlit: false,
        ..default()
    });
    
    let pizza_material = materials.add(StandardMaterial {
        base_color_texture: Some(pizza_texture.clone()),
        alpha_mode: AlphaMode::Blend,
        unlit: false,
        ..default()
    });
    
    let tree_material = materials.add(StandardMaterial {
        base_color_texture: Some(tree_texture.clone()),
        alpha_mode: AlphaMode::Blend,
        unlit: false,
        ..default()
    });
    
    let quad_mesh = meshes.add(Plane3d::default().mesh().size(1.0, 1.0));
    // Voxel materials are handled by the voxel world system
    
    commands.insert_resource(GameAssets {
        player_texture,
        pizza_texture,
        tree_texture,
        blocks_layout,
        block_material,
        player_material,
        pizza_material,
        tree_material,
        quad_mesh,
    });
}

fn setup_camera(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(25.0, 15.0, 25.0).looking_at(Vec3::ZERO, Vec3::Y),
        VoxelWorldCamera::<MyWorld>::default(),
        CameraController {
            target: Vec3::ZERO,
            distance: 30.0,
            pitch: -0.5,
            yaw: -0.5,
        },
    ));
    
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.5, -0.5, 0.0)),
    ));
}

fn setup_voxel_world() {
    // Voxel world is automatically set up by the plugin
}

fn generate_city(mut voxel_world: VoxelWorld<MyWorld>, mut commands: Commands, assets: Res<GameAssets>) {
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
                voxel_world.set_voxel(IVec3::new(x, 0, z), WorldVoxel::Solid(BlockType::Road.texture_coords() as u8));
                
                if (x % block_size == 0 && z % block_size == 0) {
                    let distance_from_center = ((x * x + z * z) as f32).sqrt();
                    if distance_from_center < 30.0 {
                        spawn_tree(&mut commands, &assets, Vec3::new(x as f32, 0.0, z as f32));
                    }
                }
            } else {
                voxel_world.set_voxel(IVec3::new(x, 0, z), WorldVoxel::Solid(BlockType::Sidewalk.texture_coords() as u8));
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
                let building_height = BUILDING_HEIGHT_MIN + 
                    ((rng() % (BUILDING_HEIGHT_MAX - BUILDING_HEIGHT_MIN) as u32) as i32);
                
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
                        if !occupied.contains_key(&(bx, bz)) {
                            occupied.insert((bx, bz), true);
                            
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
                                voxel_world.set_voxel(IVec3::new(bx, y, bz), WorldVoxel::Solid(block_type.texture_coords() as u8));
                            }
                        }
                    }
                }
            }
        }
    }
}


fn spawn_tree(c: &mut Commands, assets: &GameAssets, position: Vec3) {
    let world_pos = Vec3::new(
        position.x * BLOCK_SIZE,
        1.0,
        position.z * BLOCK_SIZE,
    );
    
    c.spawn((
        Mesh3d(assets.quad_mesh.clone()),
        MeshMaterial3d(assets.tree_material.clone()),
        Transform::from_translation(world_pos)
            .with_scale(Vec3::new(1.5, 2.0, 1.5)),
        RigidBody::Static,
        Collider::cuboid(0.3, 1.0, 0.3),
        Tree,
        FacingMode::PositionIgnoreY,
    ));
}

fn spawn_player(mut c: Commands, assets: Res<GameAssets>) {
    c.spawn((
        Mesh3d(assets.quad_mesh.clone()),
        MeshMaterial3d(assets.player_material.clone()),
        Transform::from_translation(Vec3::new(0.0, 6.0, 0.0))
            .with_scale(Vec3::new(1.0, 2.0, 1.0)),
        RigidBody::Dynamic,
        Collider::capsule(0.4, 1.0),
        LockedAxes::ROTATION_LOCKED,
        LinearDamping(2.0),
        AngularDamping(10.0),
        Friction::new(0.8),
        Restitution::new(0.1),
        Player,
        FacingMode::PositionIgnoreY,
    ));
}

fn player_movement(
    mut player_query: Query<&mut LinearVelocity, With<Player>>,
    camera_query: Query<&Transform, With<Camera3d>>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    if let (Ok(mut velocity), Ok(camera_transform)) = (player_query.single_mut(), camera_query.single()) {
        let mut direction = Vec3::ZERO;
        
        // Get camera's forward and right directions (projected onto horizontal plane)
        let camera_forward = camera_transform.forward().normalize();
        let camera_right = camera_transform.right().normalize();
        
        // Project onto horizontal plane (remove Y component)
        let forward = Vec3::new(camera_forward.x, 0.0, camera_forward.z).normalize();
        let right = Vec3::new(camera_right.x, 0.0, camera_right.z).normalize();
        
        if keyboard.pressed(KeyCode::KeyW) || keyboard.pressed(KeyCode::ArrowUp) {
            direction += forward;
        }
        if keyboard.pressed(KeyCode::KeyS) || keyboard.pressed(KeyCode::ArrowDown) {
            direction -= forward;
        }
        if keyboard.pressed(KeyCode::KeyA) || keyboard.pressed(KeyCode::ArrowLeft) {
            direction -= right;
        }
        if keyboard.pressed(KeyCode::KeyD) || keyboard.pressed(KeyCode::ArrowRight) {
            direction += right;
        }
        
        direction = direction.normalize_or_zero();
        let speed = 8.0;
        
        velocity.x = direction.x * speed;
        velocity.z = direction.z * speed;
    }
}

fn player_jump(
    mut player_query: Query<&mut LinearVelocity, With<Player>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    spatial_query: SpatialQuery,
    player_transform_query: Query<&Transform, With<Player>>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        if let (Ok(mut velocity), Ok(transform)) = (
            player_query.single_mut(),
            player_transform_query.single()
        ) {
            let ray_origin = transform.translation;
            let ray_direction = Dir3::NEG_Y;
            let max_distance = 1.1;
            
            if spatial_query.cast_ray(
                ray_origin,
                ray_direction,
                max_distance,
                true,
                &SpatialQueryFilter::default(),
            ).is_some() {
                velocity.y = 12.0;
            }
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
) {
    if keyboard.just_pressed(KeyCode::KeyE) && stats.pizzas_remaining > 0 {
        if let (Ok(player_transform), Ok(camera_transform)) = (
            player_query.single(),
            camera_query.single()
        ) {
            let throw_direction = (*camera_transform.forward() + Vec3::new(0.0, 0.3, 0.0)).normalize();
            let spawn_pos = player_transform.translation + Vec3::new(0.0, 1.0, 0.0);
            
            commands.spawn((
                Mesh3d(assets.quad_mesh.clone()),
                MeshMaterial3d(assets.pizza_material.clone()),
                Transform::from_translation(spawn_pos).with_scale(Vec3::splat(1.0)),
                RigidBody::Dynamic,
                Collider::sphere(0.2),
                LinearVelocity(throw_direction * 15.0),
                Pizza,
                FacingMode::PositionIgnoreY,
            ));
            
            stats.pizzas_remaining -= 1;
        }
    }
}

fn pizza_physics(
    pizza_query: Query<(Entity, &Transform), With<Pizza>>,
    mut commands: Commands,
    mut stats: ResMut<PlayerStats>,
) {
    for (entity, transform) in pizza_query.iter() {
        if transform.translation.y < -10.0 {
            commands.entity(entity).despawn();
        }
        
        if transform.translation.distance(Vec3::ZERO) > 100.0 {
            commands.entity(entity).despawn();
            stats.pizzas_delivered += 1;
        }
    }
}

// this is code from another game
pub fn face_camera_system(
  camera_q: Query<&Transform, With<Camera3d>>,
  mut facers_q: Query<(&mut Transform, &GlobalTransform, &FacingMode), Without<Camera3d>>
) {
  if let Ok(cam_transform) = camera_q.single() {
    for (mut transform, global, mode) in &mut facers_q {
      let billboard_pos = global.translation();
      let camera_pos = cam_transform.translation;
      
      let direction = match mode {
        FacingMode::PositionIgnoreY => {
          (camera_pos - billboard_pos).with_y(0.0).normalize()
        }
        FacingMode::Position => (camera_pos - billboard_pos).normalize(),
        FacingMode::Direction => cam_transform.forward().as_vec3()
      };
      
      // Only rotate around Y axis to keep billboards upright
      let angle = direction.z.atan2(direction.x);
      let rotation = Quat::from_rotation_y(angle - std::f32::consts::FRAC_PI_2);
      
      // Preserve position and scale
      let old_scale = transform.scale;
      let old_translation = transform.translation;
      transform.rotation = rotation;
      transform.scale = old_scale;
      transform.translation = old_translation;
    }
  }
}

fn camera_follow_player(
    player_query: Query<&Transform, With<Player>>,
    mut camera_query: Query<&mut CameraController>,
) {
    if let (Ok(player_transform), Ok(mut controller)) = (
        player_query.single(),
        camera_query.single_mut()
    ) {
        controller.target = player_transform.translation + Vec3::new(0.0, 2.0, 0.0);
    }
}

fn camera_controller(
    mut camera_query: Query<(&mut Transform, &mut CameraController), With<Camera3d>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
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
        if keyboard.pressed(KeyCode::Equal) {
            controller.distance = (controller.distance - 10.0 * time.delta_secs()).max(5.0);
        }
        if keyboard.pressed(KeyCode::Minus) {
            controller.distance = (controller.distance + 10.0 * time.delta_secs()).min(50.0);
        }
        
        let rotation = Quat::from_euler(EulerRot::YXZ, controller.yaw, controller.pitch, 0.0);
        let offset = rotation * Vec3::new(0.0, 0.0, controller.distance);
        
        transform.translation = controller.target + offset;
        transform.look_at(controller.target, Vec3::Y);
    }
}

fn mouse_look(
    mut camera_query: Query<&mut CameraController>,
    mut mouse_motion: EventReader<MouseMotion>,
    mouse_button: Res<ButtonInput<MouseButton>>,
) {
    if let Ok(mut controller) = camera_query.single_mut() {
        if mouse_button.pressed(MouseButton::Right) {
            for event in mouse_motion.read() {
                controller.yaw -= event.delta.x * 0.003;
                controller.pitch = (controller.pitch - event.delta.y * 0.003).clamp(-1.5, 0.0);
            }
        }
    }
}

fn billboard_system(
    mut billboard_query: Query<&mut Transform, (With<Billboard>, Without<Camera3d>)>,
    camera_query: Query<&Transform, (With<Camera3d>, Without<Billboard>)>,
) {
    if let Ok(camera_transform) = camera_query.single() {
        for mut transform in billboard_query.iter_mut() {
            let position = transform.translation;
            let camera_pos = camera_transform.translation;
            
            // Calculate direction from billboard to camera
            let direction = (camera_pos - position).normalize();
            
            // Create rotation to face camera, constrained to Y axis
            let angle = direction.z.atan2(direction.x);
            let rotation = Quat::from_rotation_y(angle - std::f32::consts::FRAC_PI_2);
            
            // Apply rotation while preserving position and scale
            let old_scale = transform.scale;
            let old_translation = transform.translation;
            transform.rotation = rotation;
            transform.scale = old_scale;
            transform.translation = old_translation;
        }
    }
}

fn add_colliders_to_chunks(
    mut commands: Commands,
    mesh_query: Query<(Entity, &Mesh3d), Without<Collider>>,
    meshes: Res<Assets<Mesh>>,
) {
    for (entity, mesh3d) in mesh_query.iter() {
        if let Some(mesh) = meshes.get(&mesh3d.0) {
            // Check if mesh has vertices before creating trimesh collider
            if let Some(positions) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
                if !positions.is_empty() {
                    if let Some(collider) = Collider::trimesh_from_mesh(mesh) {
                        commands.entity(entity).insert((
                            RigidBody::Static,
                            collider,
                        ));
                    }
                }
            }
        }
    }
}

fn update_ui(
    mut commands: Commands,
    stats: Res<PlayerStats>,
    ui_root_query: Query<Entity, With<Node>>,
) {
    for entity in ui_root_query.iter() {
        commands.entity(entity).despawn();
    }
    
    commands.spawn(Node::default()).with_children(|parent| {
        parent.spawn((
            Text::new(format!("Pizzas Remaining: {}", stats.pizzas_remaining)),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            TextColor(Color::WHITE),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(10.0),
                left: Val::Px(10.0),
                ..default()
            },
        ));
        
        parent.spawn((
            Text::new(format!("Pizzas Delivered: {}", stats.pizzas_delivered)),
            TextFont {
                font_size: 24.0,
                ..default()
            },
            TextColor(Color::WHITE),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(40.0),
                left: Val::Px(10.0),
                ..default()
            },
        ));
        
        parent.spawn((
            Text::new("Controls: WASD = Move, Space = Jump, E = Throw Pizza, Right Click + Mouse = Look, QX = Rotate, ZC = Pitch, +- = Zoom"),
            TextFont {
                font_size: 16.0,
                ..default()
            },
            TextColor(Color::WHITE),
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(10.0),
                left: Val::Px(10.0),
                ..default()
            },
        ));
    });
}
