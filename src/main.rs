#![feature(let_chains)]
#![allow(dead_code)]

use bevy::asset::AssetMetaCheck;
use bevy::prelude::*;
use avian3d::prelude::*;
use bevy_embedded_assets::EmbeddedAssetPlugin;
use enum_assoc::Assoc;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Assoc)]
#[func(const fn texture_coords(self) -> u32)]
enum BlockType {
    #[assoc(texture_coords = 0)]
    Grass,
    #[assoc(texture_coords = 1)]
    GrayBricks,
    #[assoc(texture_coords = 2)]
    Pavement,
    #[assoc(texture_coords = 3)]
    Window,
    #[assoc(texture_coords = 4)]
    RedBricks,
}

impl BlockType {
    const TEXTURE_SIZE: f32 = 16.0;
    
    fn get_texture_atlas_index(self) -> usize {
        let (x, y) = self.texture_coords();
        (y * 4 + x) as usize
    }
}

const CITY_SIZE: i32 = 50;
const BLOCK_SIZE: f32 = 1.0;
const BUILDING_HEIGHT_MIN: i32 = 3;
const BUILDING_HEIGHT_MAX: i32 = 8;

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

#[derive(Resource)]
struct GameAssets {
    blocks_texture: Handle<Image>,
    player_texture: Handle<Image>,
    pizza_texture: Handle<Image>,
    blocks_layout: Handle<TextureAtlasLayout>,
    block_material: Handle<StandardMaterial>,
    player_material: Handle<StandardMaterial>,
    pizza_material: Handle<StandardMaterial>,
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
        }))
        .add_plugins(EmbeddedAssetPlugin::default())
        .add_plugins(PhysicsPlugins::default())
        .insert_resource(PlayerStats {
            pizzas_delivered: 0,
            pizzas_remaining: 10,
        })
        .add_systems(Startup, (setup_assets, setup_camera, generate_city, spawn_player).chain())
        .add_systems(Update, (
            player_movement,
            player_jump,
            throw_pizza,
            pizza_physics,
            camera_follow_player,
            billboard_system,
            camera_controller,
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
    let blocks_texture = asset_server.load("blocks.png");
    let player_texture = asset_server.load("guy.png");
    let pizza_texture = asset_server.load("pizzabox.png");
    
    let layout = TextureAtlasLayout::from_grid(
        UVec2::splat(BlockType::TEXTURE_SIZE as u32),
        4,
        2,
        None,
        None,
    );
    let blocks_layout = texture_atlas_layouts.add(layout);
    
    let block_material = materials.add(StandardMaterial {
        base_color_texture: Some(blocks_texture.clone()),
        unlit: false,
        ..default()
    });
    
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
    
    let quad_mesh = meshes.add(Plane3d::default().mesh().size(1.0, 1.0));
    
    commands.insert_resource(GameAssets {
        blocks_texture,
        player_texture,
        pizza_texture,
        blocks_layout,
        block_material,
        player_material,
        pizza_material,
        quad_mesh,
    });
}

fn setup_camera(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(25.0, 15.0, 25.0).looking_at(Vec3::ZERO, Vec3::Y),
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

fn generate_city(mut commands: Commands, assets: Res<GameAssets>) {
    use std::collections::HashMap;
    
    let mut noise_seed = 12345u32;
    let mut rng = || {
        noise_seed = noise_seed.wrapping_mul(1103515245).wrapping_add(12345);
        noise_seed
    };
    
    let mut _height_map = HashMap::new();
    
    for x in -CITY_SIZE..CITY_SIZE {


        for z in -CITY_SIZE..CITY_SIZE {
            let distance_from_center = ((x * x + z * z) as f32).sqrt();
            let is_road = (x % 8 == 0) || (z % 8 == 0);
            
            if is_road {
                spawn_block(&mut commands, &assets, IVec3::new(x, 0, z), BlockType::Road);
                
                if (x % 8 == 0 && z % 8 == 0) && distance_from_center < 30.0 {
                    for y in 1..3 {
                        spawn_block(&mut commands, &assets, IVec3::new(x, y, z), BlockType::Tree);
                    }
                }
            } else {
                spawn_block(&mut commands, &assets, IVec3::new(x, 0, z), BlockType::Sidewalk);
                
                if distance_from_center < 35.0 && (rng() % 100) < 40 {
                    let building_height = BUILDING_HEIGHT_MIN + 
                        ((rng() % (BUILDING_HEIGHT_MAX - BUILDING_HEIGHT_MIN) as u32) as i32);
                    _height_map.insert((x, z), building_height);
                    
                    for y in 1..=building_height {
                        let block_type = if y == building_height {
                            BlockType::Roof
                        } else if y == 1 && (rng() % 4) == 0 {
                            BlockType::Door
                        } else if (rng() % 3) == 0 {
                            BlockType::Window
                        } else {
                            BlockType::Building
                        };
                        spawn_block(&mut commands, &assets, IVec3::new(x, y, z), block_type);
                    }
                }
            }
        }
    }
}

fn spawn_block(c: &mut Commands, assets: &GameAssets, position: IVec3, block_type: BlockType) {
    let world_pos = Vec3::new(
        position.x as f32 * BLOCK_SIZE,
        position.y as f32 * BLOCK_SIZE,
        position.z as f32 * BLOCK_SIZE,
    );
    
    c.spawn((
        Mesh3d(assets.quad_mesh.clone()),
        MeshMaterial3d(assets.block_material.clone()),
        Transform::from_translation(world_pos),
        RigidBody::Static,
        Collider::cuboid(BLOCK_SIZE * 0.5, BLOCK_SIZE * 0.5, BLOCK_SIZE * 0.5),
        CityBlock { block_type, position },
        Billboard,
    ));
}

fn spawn_player(mut c: Commands, assets: Res<GameAssets>) {
    c.spawn((
        Mesh3d(assets.quad_mesh.clone()),
        MeshMaterial3d(assets.player_material.clone()),
        Transform::from_translation(Vec3::new(0.0, 2.0, 0.0)),
        RigidBody::Dynamic,
        Collider::capsule(0.3, 0.8),
        LockedAxes::ROTATION_LOCKED,
        Player,
        Billboard,
    ));
}

fn player_movement(
    mut player_query: Query<&mut LinearVelocity, With<Player>>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    if let Ok(mut velocity) = player_query.single_mut() {
        let mut direction = Vec3::ZERO;
        
        if keyboard.pressed(KeyCode::KeyW) || keyboard.pressed(KeyCode::ArrowUp) {
            direction.z -= 1.0;
        }
        if keyboard.pressed(KeyCode::KeyS) || keyboard.pressed(KeyCode::ArrowDown) {
            direction.z += 1.0;
        }
        if keyboard.pressed(KeyCode::KeyA) || keyboard.pressed(KeyCode::ArrowLeft) {
            direction.x -= 1.0;
        }
        if keyboard.pressed(KeyCode::KeyD) || keyboard.pressed(KeyCode::ArrowRight) {
            direction.x += 1.0;
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
                Transform::from_translation(spawn_pos).with_scale(Vec3::splat(0.5)),
                RigidBody::Dynamic,
                Collider::sphere(0.2),
                LinearVelocity(throw_direction * 15.0),
                Pizza,
                Billboard,
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
    mut camera_query: Query<(&mut Transform, &CameraController), With<Camera3d>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
) {
    if let Ok((mut transform, controller)) = camera_query.single_mut() {
        let mut yaw = controller.yaw;
        let mut pitch = controller.pitch;
        let mut distance = controller.distance;
        
        if keyboard.pressed(KeyCode::KeyQ) {
            yaw -= 2.0 * time.delta_secs();
        }
        if keyboard.pressed(KeyCode::KeyX) {
            yaw += 2.0 * time.delta_secs();
        }
        if keyboard.pressed(KeyCode::KeyZ) {
            pitch = (pitch - 1.0 * time.delta_secs()).clamp(-1.5, 0.0);
        }
        if keyboard.pressed(KeyCode::KeyC) {
            pitch = (pitch + 1.0 * time.delta_secs()).clamp(-1.5, 0.0);
        }
        if keyboard.pressed(KeyCode::Equal) {
            distance = (distance - 10.0 * time.delta_secs()).max(5.0);
        }
        if keyboard.pressed(KeyCode::Minus) {
            distance = (distance + 10.0 * time.delta_secs()).min(50.0);
        }
        
        let rotation = Quat::from_euler(EulerRot::YXZ, yaw, pitch, 0.0);
        let offset = rotation * Vec3::new(0.0, 0.0, distance);
        
        transform.translation = controller.target + offset;
        transform.look_at(controller.target, Vec3::Y);
    }
}

fn billboard_system(
    mut billboard_query: Query<&mut Transform, (With<Billboard>, Without<Camera3d>)>,
    camera_query: Query<&Transform, (With<Camera3d>, Without<Billboard>)>,
) {
    if let Ok(camera_transform) = camera_query.single() {
        for mut transform in billboard_query.iter_mut() {
            let direction = (camera_transform.translation - transform.translation).normalize();
            let position = transform.translation;
            transform.look_at(position + direction, Vec3::Y);
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
            Text::new("Controls: WASD = Move, Space = Jump, E = Throw Pizza, QX = Rotate Camera, ZC = Pitch, +- = Zoom"),
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
