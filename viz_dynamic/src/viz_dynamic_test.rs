use r2r::geometry_msgs::msg::Transform;
use r2r::geometry_msgs::msg::Vector3;
use r2r::std_msgs::msg::ColorRGBA;
use r2r::viz_tools_msgs::srv::ManipulateDynamicMarker;


#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "viz_dynamic_client", "")?;

    let client = node.create_client::<ManipulateDynamicMarker::Service>("manipulate_dynamic_marker")?;
    let waiting = node.is_available(&client)?;

    let handle = std::thread::spawn(move || loop {
        &node.spin_once(std::time::Duration::from_millis(100));
    });

    r2r::log_warn!("viz_dynamic_client", "Waiting for Dinamic Visualization Service...");
    waiting.await?;
    r2r::log_info!("viz_dynamic_client", "Dinamic Visualization Service available.");
    r2r::log_info!(
        "viz_dynamic_client",
        "Dinamic Visualization Test Node started."
    );

    std::thread::sleep(std::time::Duration::from_millis(1000));

    let mut messages = vec![];

    let message_1 = ManipulateDynamicMarker::Request {
        command: "update".to_string(),
        child_id: "cube_1".to_string(),
        parent_id: "input".to_string(),
        transformation: Transform::default(),
        use_primitive: true,
        primitive_type: 1,
        scale: Vector3 {
            x: 0.1,
            y: 0.1,
            z: 0.1
        },
        color: ColorRGBA {
            r: 1.0,
            g: 0.0,
            b: 0.0,
            a: 1.0
        },
        ..ManipulateDynamicMarker::Request::default()
    };
    messages.push(message_1);

    let message_2 = ManipulateDynamicMarker::Request {
        command: "update".to_string(),
        child_id: "cyl_1".to_string(),
        parent_id: "output".to_string(),
        transformation: Transform::default(),
        use_primitive: true,
        primitive_type: 2,
        scale: Vector3 {
            x: 0.1,
            y: 0.1,
            z: 0.1
        },
        color: ColorRGBA {
            r: 0.0,
            g: 1.0,
            b: 0.0,
            a: 1.0
        },
        ..ManipulateDynamicMarker::Request::default()
    };
    messages.push(message_2);

    let message_3 = ManipulateDynamicMarker::Request {
        command: "remove".to_string(),
        child_id: "cube_1".to_string(),
        parent_id: "input".to_string(),
        transformation: Transform::default(),
        use_primitive: true,
        primitive_type: 1,
        scale: Vector3 {
            x: 0.1,
            y: 0.1,
            z: 0.1
        },
        color: ColorRGBA {
            r: 1.0,
            g: 0.0,
            b: 0.0,
            a: 1.0
        },
        ..ManipulateDynamicMarker::Request::default()
    };
    messages.push(message_3);

    let message_4 = ManipulateDynamicMarker::Request {
        command: "update".to_string(),
        child_id: "cube_6".to_string(),
        parent_id: "input".to_string(),
        transformation: Transform::default(),
        use_primitive: true,
        primitive_type: 1,
        scale: Vector3 {
            x: 0.1,
            y: 0.1,
            z: 0.1
        },
        color: ColorRGBA {
            r: 0.0,
            g: 0.0,
            b: 1.0,
            a: 1.0
        },
        ..ManipulateDynamicMarker::Request::default()
    };
    messages.push(message_4);

    let message_5 = ManipulateDynamicMarker::Request {
        command: "clear".to_string(),
        ..ManipulateDynamicMarker::Request::default()
    };
    messages.push(message_5);

    for message in messages {
        sms_test(
            &client,
            message
        ).await?;
    }
    
    handle.join().unwrap();

    Ok(())
}

async fn sms_test(
    client: &r2r::Client<ManipulateDynamicMarker::Service>,
    message: ManipulateDynamicMarker::Request,
) -> Result<(), Box<dyn std::error::Error>> {

    r2r::log_info!("viz_dynamic_client", "Will now send Request to Dynamic Visualization.");

    let response = client
        .request(&message)
        .expect("Could not send Dynamic Visualization request.")
        .await
        .expect("Cancelled.");

    r2r::log_info!("viz_dynamic_client", "Request to Dynamic Visualization sent.");

    match response.success {
        true => {
            r2r::log_info!("viz_dynamic_client", "Got Dynamic Visualization response: {}", response.success);
        }
        false => {
            r2r::log_error!(
                "viz_dynamic_client",
                "Couldn't Manipulate Dynamic Visualization for command command '{}'.",
                message.command
            );
        }
    }

    std::thread::sleep(std::time::Duration::from_millis(3000));

    Ok(())
}