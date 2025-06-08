use btleplug::api::{
    Central, Manager as _, Peripheral as _, ScanFilter, ValueNotification, CharPropFlags,
};
use btleplug::platform::Manager;
use tokio::fs::File;
use tokio::io::AsyncWriteExt;
use futures::stream::StreamExt;
use std::error::Error;
use tokio::time::{Instant, Duration};
use chrono::Local;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    let adapter = adapters.into_iter().nth(0).expect("No Bluetooth adapter found");
    let mut device = None;

    adapter.start_scan(ScanFilter::default()).await?;
    println!("Scanning for 5s...");
    tokio::time::sleep(Duration::from_secs(5)).await;

    let peripherals = adapter.peripherals().await?;
    for p in peripherals {
        if let Ok(Some(props)) = p.properties().await {
            if let Some(name) = &props.local_name {
                if name.to_lowercase().contains("nrf") {
                    device = Some(p);
                    break;
                }
            }
        }
    }

    let device = device.expect("nRF device not found");
    device.connect().await?;
    device.discover_services().await?;
    println!("Connected");

    let chars = device.characteristics();
    let notify_char = chars.iter()
        .find(|c| c.properties.contains(CharPropFlags::NOTIFY))
        .expect("No notifiable characteristic found");

    device.subscribe(notify_char).await?;

    let mut file = File::create("nrf_emg_100s_6.csv").await?;
    println!("Receiving data for 20s...");

    let start = Instant::now();
    let mut notifications = device.notifications().await?;

    while Instant::now().duration_since(start) < Duration::from_secs(100) {
        if let Some(ValueNotification { value, .. }) = notifications.next().await {
            if let Ok(data_str) = std::str::from_utf8(&value) {
                let timestamp = Local::now().format("%Y-%m-%d %H:%M:%S%.3f").to_string();
                let line = format!("{},{}\n", timestamp, data_str.trim());  // trim removes any newline
                file.write_all(line.as_bytes()).await?;
            } else {
                eprintln!("Received non-UTF8 data");
            }
        }
    }

    println!("Finished. Data written to ble_stream.csv.");
    Ok(())
}
