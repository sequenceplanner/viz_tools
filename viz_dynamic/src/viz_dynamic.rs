use futures::stream::Stream;
use futures::Future;
use futures::StreamExt;
use r2r::builtin_interfaces::msg::Duration;
use r2r::geometry_msgs::msg::Point;
use r2r::geometry_msgs::msg::Pose;
use r2r::geometry_msgs::msg::Quaternion;
use r2r::geometry_msgs::msg::Transform;
use r2r::geometry_msgs::msg::Vector3;
use r2r::std_msgs::msg::ColorRGBA;
use r2r::std_msgs::msg::Header;
use r2r::tf_tools_msgs::srv::ManipulateScene;
use r2r::visualization_msgs::msg::Marker;
use r2r::visualization_msgs::msg::MarkerArray;
use r2r::viz_tools_msgs::srv::ManipulateDynamicMarker;
use std::path::Path;
use std::sync::{Arc, Mutex};

#[derive(Clone)]
pub struct MarkerData {
    pub child_id: String,
    pub parent_id: String,
    pub tf: Transform,
    pub color: ColorRGBA,
    pub scale: Vector3,
    pub use_primitive: bool,
    pub primitive_type: i32,
    pub absolute_mesh_path: String,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "dynamic_visualization", "")?;

    // let clock = r2r::Clock::create(r2r::ClockType::RosTime)?;

    let timer = node.create_wall_timer(std::time::Duration::from_millis(50))?;
    let marker_publisher =
        node.create_publisher::<r2r::visualization_msgs::msg::MarkerArray>("dynamic_markers")?;

    let server_requests =
        node.create_service::<ManipulateDynamicMarker::Service>("manipulate_dynamic_marker")?;
    let sms_client = node.create_client::<ManipulateScene::Service>("manipulate_scene")?;

    let waiting_for_sms_server = node.is_available(&sms_client)?;
    let markers = Arc::new(Mutex::new(Vec::<MarkerData>::default()));
    let markers_clone_1 = markers.clone();
    let markers_clone_2 = markers.clone();

    tokio::task::spawn(async move {
        match marker_publisher_callback(marker_publisher, markers_clone_1, timer).await {
            Ok(()) => println!("done."),
            Err(e) => println!("error: {}", e),
        };
    });

    tokio::task::spawn(async move {
        let result = dynamic_visualization_server(
            server_requests,
            markers_clone_2,
            sms_client,
            waiting_for_sms_server,
        )
        .await;
        match result {
            Ok(()) => r2r::log_info!(
                "dynamic_visualization",
                "Cube Handler Service call succeeded."
            ),
            Err(e) => r2r::log_error!(
                "dynamic_visualization",
                "Cube Handler Service call failed with: {}.",
                e
            ),
        };
    });

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    handle.join().unwrap();

    Ok(())
}

async fn dynamic_visualization_server(
    mut requests: impl Stream<Item = r2r::ServiceRequest<ManipulateDynamicMarker::Service>> + Unpin,
    mut marker_datas: Arc<Mutex<Vec<MarkerData>>>,
    sms_client: r2r::Client<ManipulateScene::Service>,
    waiting_for_sms_server: impl Future<Output = r2r::Result<()>>,
) -> Result<(), Box<dyn std::error::Error>> {
    r2r::log_warn!(
        "dynamic_visualization",
        "Waiting for Scene Manipulation Service..."
    );
    waiting_for_sms_server.await?;
    r2r::log_info!(
        "dynamic_visualization",
        "Scene Manipulation Service available."
    );
    r2r::log_info!(
        "dynamic_visualization",
        "Dynamic Visualization Service node started."
    );

    loop {
        match requests.next().await {
            Some(request) => {
                match check_message(&request.message) {
                    true => (),
                    false => {
                        request
                            .respond(ManipulateDynamicMarker::Response { success: false })
                            .expect("Could not send service response.");
                        continue;
                    }
                }
                match request.message.command.as_str() {
                    "update" => {
                        match manipulate_scene(&request.message, &sms_client).await {
                            true => match get_marker_data(&request.message.child_id, &marker_datas)
                            {
                                Some(md) => {
                                    match update_marker(&request.message, &marker_datas, &md) {
                                        true => {
                                            r2r::log_info!(
                                                "dynamic_visualization",
                                                "Updated marker: {}",
                                                request.message.child_id
                                            );
                                            request
                                                .respond(ManipulateDynamicMarker::Response {
                                                    success: true,
                                                })
                                                .expect("Could not send service response.");
                                            continue;
                                        }
                                        false => {
                                            r2r::log_error!(
                                                "dynamic_visualization",
                                                "Failed to update marker: {}",
                                                request.message.child_id
                                            );
                                            request
                                                .respond(ManipulateDynamicMarker::Response {
                                                    success: false,
                                                })
                                                .expect("Could not send service response.");
                                            continue;
                                        }
                                    }
                                }
                                None => match add_marker(&request.message, &mut marker_datas) {
                                    true => {
                                        r2r::log_info!(
                                            "dynamic_visualization",
                                            "Added marker: {}",
                                            request.message.child_id
                                        );
                                        request
                                            .respond(ManipulateDynamicMarker::Response {
                                                success: true,
                                            })
                                            .expect("Could not send service response.");
                                        continue;
                                    }
                                    false => {
                                        r2r::log_error!(
                                            "dynamic_visualization",
                                            "Failed to add marker: {}",
                                            request.message.child_id
                                        );
                                        request
                                            .respond(ManipulateDynamicMarker::Response {
                                                success: false,
                                            })
                                            .expect("Could not send service response.");
                                        continue;
                                    }
                                },
                            },
                            false => {
                                r2r::log_error!(
                                    "dynamic_visualization",
                                    "SMS falied to update a frame, look at the logs from SMS."
                                );
                                request
                                    .respond(ManipulateDynamicMarker::Response { success: false })
                                    .expect("Could not send service response.");
                                continue;
                            }
                        };
                    }
                    "remove" => match manipulate_scene(&request.message, &sms_client).await {
                        true => match get_marker_data(&request.message.child_id, &marker_datas) {
                            Some(_) => match remove_marker(&request.message, &marker_datas) {
                                true => {
                                    r2r::log_info!(
                                        "dynamic_visualization",
                                        "Removed marker: {}",
                                        request.message.child_id
                                    );
                                    request
                                        .respond(ManipulateDynamicMarker::Response {
                                            success: true,
                                        })
                                        .expect("Could not send service response.");
                                    continue;
                                }
                                false => {
                                    request
                                        .respond(ManipulateDynamicMarker::Response {
                                            success: false,
                                        })
                                        .expect("Could not send service response.");
                                    continue;
                                }
                            },
                            None => {
                                r2r::log_warn!(
                                    "dynamic_visualization",
                                    "Can't remove nonexisting marker: {}",
                                    request.message.child_id
                                );
                                request
                                    .respond(ManipulateDynamicMarker::Response { success: false })
                                    .expect("Could not send service response.");
                                continue;
                            }
                        },
                        false => {
                            r2r::log_error!(
                                "dynamic_visualization",
                                "SMS falied to remove a frame, look at the logs from SMS."
                            );
                            request
                                .respond(ManipulateDynamicMarker::Response { success: false })
                                .expect("Could not send service response.");
                            continue;
                        }
                    },
                    "clear" => {
                        let mut overall_success: Vec<bool> = vec![];
                        for name in get_all_marker_names(&marker_datas) {
                            let message = ManipulateDynamicMarker::Request {
                                command: "remove".to_string(),
                                child_id: name.to_string(),
                                ..ManipulateDynamicMarker::Request::default()
                            };
                            match manipulate_scene(&message, &sms_client).await {
                                true => match remove_marker(&message, &marker_datas) {
                                    true => {
                                        r2r::log_info!(
                                            "dynamic_visualization",
                                            "Removed marker: {}",
                                            message.child_id
                                        );
                                        overall_success.push(true);
                                    }
                                    false => {
                                        r2r::log_warn!(
                                            "dynamic_visualization",
                                            "Failed to remove marker: {}",
                                            message.child_id
                                        );
                                        overall_success.push(false);
                                    }
                                },
                                false => {
                                    r2r::log_error!(
                                        "dynamic_visualization",
                                        "SMS falied to remove a frame, look at the logs from SMS."
                                    );
                                    overall_success.push(false);
                                }
                            }
                        }
                        match overall_success.iter().all(|x| *x) {
                            true => {
                                request
                                    .respond(ManipulateDynamicMarker::Response { success: true })
                                    .expect("Could not send service response.");
                                continue;
                            }
                            false => {
                                request
                                    .respond(ManipulateDynamicMarker::Response { success: false })
                                    .expect("Could not send service response.");
                                continue;
                            }
                        }
                    }
                    _ => {
                        r2r::log_error!(
                            "dynamic_visualization",
                            "Unknown Command: {}",
                            request.message.command.as_str()
                        );
                        request
                            .respond(ManipulateDynamicMarker::Response { success: false })
                            .expect("Could not send service response.");
                        continue;
                    }
                }
            }
            None => (),
        }
    }
}

fn get_marker_data(
    child_id: &str,
    marker_datas: &Arc<Mutex<Vec<MarkerData>>>,
) -> Option<MarkerData> {
    marker_datas
        .lock()
        .unwrap()
        .iter()
        .find(|x| x.child_id == child_id)
        .map(|x| x.clone())
}

fn get_all_marker_names(marker_datas: &Arc<Mutex<Vec<MarkerData>>>) -> Vec<String> {
    marker_datas
        .lock()
        .unwrap()
        .iter()
        .map(|x| x.child_id.to_string())
        .collect()
}

fn add_marker(
    message: &r2r::viz_tools_msgs::srv::ManipulateDynamicMarker::Request,
    marker_datas: &Arc<Mutex<Vec<MarkerData>>>,
) -> bool {
    match marker_datas.lock() {
        Ok(mut x) => {
            x.push(make_marker_data(&message));
            true
        }
        Err(e) => {
            r2r::log_error!("dynamic_visualization", "Error unwrapping: {}", e);
            false
        }
    }
}

fn check_message(message: &r2r::viz_tools_msgs::srv::ManipulateDynamicMarker::Request) -> bool {
    if message.child_id == "" {
        r2r::log_warn!("dynamic_visualization", "Child frame ID can't be empty.",);
        return false;
    }

    if message.parent_id == "" {
        r2r::log_warn!("dynamic_visualization", "Parent frame ID can't be empty.",);
        return false;
    }

    if message.child_id == message.parent_id {
        r2r::log_warn!(
            "dynamic_visualization",
            "Parent frame ID can't be equal to Child frame ID.",
        );
        return false;
    }
    if !message.use_primitive {
        if !Path::new(message.absolute_mesh_path.as_str()).exists() {
            r2r::log_warn!("dynamic_visualization", "Invalid mesh path.",);
            return false;
        }
    }

    true
}

fn update_marker(
    message: &r2r::viz_tools_msgs::srv::ManipulateDynamicMarker::Request,
    marker_datas: &Arc<Mutex<Vec<MarkerData>>>,
    new_marker: &MarkerData,
) -> bool {
    let old_marker_index = match get_marker_data(&message.child_id, marker_datas) {
        Some(_) => match marker_datas.lock() {
            Ok(y) => {
                match y
                    .iter()
                    .position(|z| *z.child_id == message.child_id)
                    .map(|m| m as i32)
                {
                    Some(n) => n,
                    None => -1,
                }
            }
            Err(e) => {
                r2r::log_error!("dynamic_visualization", "Error unwrapping: {}", e);
                -1
            }
        },
        None => -1,
    };

    match old_marker_index >= 0 {
        true => match marker_datas.lock() {
            Ok(mut x) => {
                x.remove(old_marker_index as usize);
                x.push(new_marker.clone());
                true
            }
            Err(e) => {
                r2r::log_error!("dynamic_visualization", "Error unwrapping: {}", e);
                false
            }
        },
        false => false,
    }
}

fn remove_marker(
    message: &r2r::viz_tools_msgs::srv::ManipulateDynamicMarker::Request,
    marker_datas: &Arc<Mutex<Vec<MarkerData>>>,
) -> bool {
    let marker_index = match get_marker_data(&message.child_id, marker_datas) {
        Some(_) => match marker_datas.lock() {
            Ok(y) => {
                match y
                    .iter()
                    .position(|z| *z.child_id == message.child_id)
                    .map(|m| m as i32)
                {
                    Some(n) => n,
                    None => -1,
                }
            }
            Err(e) => {
                r2r::log_error!("dynamic_visualization", "Error unwrapping: {}", e);
                -1
            }
        },
        None => -1,
    };

    match marker_index >= 0 {
        true => match marker_datas.lock() {
            Ok(mut x) => {
                x.remove(marker_index as usize);
                true
            }
            Err(e) => {
                r2r::log_error!("dynamic_visualization", "Error unwrapping: {}", e);
                false
            }
        },
        false => false,
    }
}

fn make_marker_data(
    message: &r2r::viz_tools_msgs::srv::ManipulateDynamicMarker::Request,
) -> MarkerData {
    MarkerData {
        child_id: message.child_id.to_string(),
        parent_id: message.parent_id.to_string(),
        tf: message.transformation.clone(),
        color: message.color.clone(),
        scale: message.scale.clone(),
        use_primitive: message.use_primitive,
        primitive_type: message.primitive_type,
        absolute_mesh_path: message.absolute_mesh_path.to_string(),
    }
}

async fn marker_publisher_callback(
    publisher: r2r::Publisher<MarkerArray>,
    marker_datas: Arc<Mutex<Vec<MarkerData>>>,
    // mut clock: r2r::Clock,
    mut timer: r2r::Timer,
) -> Result<(), Box<dyn std::error::Error>> {
    // let now = clock.get_now().expect("Could not get ROS time.");

    loop {
        let mut markers: Vec<Marker> = vec![];
        let mut id = 0;
        for marker in &*marker_datas.lock().unwrap() {
            id = id + 1;
            let indiv_marker = Marker {
                header: Header {
                    stamp: r2r::builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
                    frame_id: marker.child_id.to_string(),
                },
                ns: "".to_string(),
                id,
                type_: match marker.use_primitive {
                    true => match marker.primitive_type {
                        1 => 1,
                        2 => 2,
                        3 => 3,
                        x => {
                            r2r::log_error!(
                                "dynamic_visualization",
                                "Invalid primitive marker type: {}",
                                x
                            );
                            r2r::log_error!(
                                "dynamic_visualization",
                                "Using 'cube' as primitive type"
                            );
                            1
                        }
                    },
                    false => 10,
                },
                action: 0,
                pose: Pose {
                    position: Point {
                        x: marker.tf.translation.x,
                        y: marker.tf.translation.y,
                        z: marker.tf.translation.z,
                    },
                    orientation: Quaternion {
                        x: marker.tf.rotation.x,
                        y: marker.tf.rotation.y,
                        z: marker.tf.rotation.z,
                        w: marker.tf.rotation.w,
                    },
                },
                lifetime: Duration { sec: 1, nanosec: 0 },
                scale: Vector3 {
                    x: marker.scale.x,
                    y: marker.scale.y,
                    z: marker.scale.z,
                },
                color: ColorRGBA {
                    r: marker.color.r,
                    g: marker.color.g,
                    b: marker.color.b,
                    a: marker.color.a,
                },
                mesh_resource: match marker.use_primitive {
                    true => "".to_string(),
                    false => format!("file://{}", marker.absolute_mesh_path),
                },
                ..Marker::default()
            };
            markers.push(indiv_marker)
        }

        let array_msg = MarkerArray { markers: markers };

        match publisher.publish(&array_msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    "dynamic_visualization",
                    "Publisher failed to send a message with: {}",
                    e
                );
            }
        };
        timer.tick().await?;
    }
}

async fn manipulate_scene(
    message: &r2r::viz_tools_msgs::srv::ManipulateDynamicMarker::Request,
    sms_client: &r2r::Client<ManipulateScene::Service>,
) -> bool {
    r2r::log_info!("dynamic_visualization", "Sending request to SMS.");

    let sms_request = ManipulateScene::Request {
        command: message.command.to_string(),
        child_frame: message.child_id.to_string(),
        parent_frame: message.parent_id.to_string(),
        transform: message.transformation.clone(),
        same_position_in_world: false,
    };

    let sms_response = sms_client
        .request(&sms_request)
        .expect("Could not send SMS request.")
        .await
        .expect("Cancelled.");

    r2r::log_info!("dynamic_visualization", "Sending request to SMS.");

    sms_response.success
}
