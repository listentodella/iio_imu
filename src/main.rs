use chrono::{DateTime, offset::Utc};
use iio::{channel, context, device};
use industrial_io as iio;
use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, SystemTime},
};

fn main() {
    let mut trigs = Vec::new();
    let ctx = context::Context::new().unwrap();
    let num = ctx.num_devices();
    println!("ctx = {:?} has {} devices", ctx.name(), num);

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        println!("Try to exit......");
        r.store(false, Ordering::SeqCst);
    })
    .unwrap();

    let devices = ctx.devices();
    for dev in devices {
        if dev.is_trigger() {
            println!("detect a trigger device {:?}", dev.name());
            trigs.push(dev);
        }
    }

    // let dev = ctx.find_device("or device name").unwrap();
    let dev = ctx.find_device("iio:device1").unwrap();

    println!(
        "get device {:?}, channels {}",
        dev.name(),
        dev.num_channels()
    );

    let trigger = trigs.pop().unwrap();

    let _ = dev.remove_trigger();
    dev.set_trigger(&trigger).unwrap();

    for c in dev.channels() {
        println!("get channel = {:?}", c.name());
        println!("attrs:{:?}", c.attr_read_all());
        if c.is_scan_element() {
            c.enable();
            if c.is_enabled() {
                println!("enabled success");
            } else {
                println!("enabled fail");
            }
        }

        if c.has_attr("sampling_frequency") {
            c.attr_write_float("sampling_frequency", 25.0f64).unwrap();
        }
    }
    let ts_chan = dev.find_input_channel("timestamp").unwrap();
    let acc_x_chan = dev.find_input_channel("accel_x").unwrap();
    let acc_y_chan = dev.find_input_channel("accel_y").unwrap();
    let acc_z_chan = dev.find_input_channel("accel_z").unwrap();
    let gyr_x_chan = dev.find_input_channel("anglvel_x").unwrap();
    let gyr_y_chan = dev.find_input_channel("anglvel_y").unwrap();
    let gyr_z_chan = dev.find_input_channel("anglvel_z").unwrap();

    // acc_x_chan.disable();
    // acc_y_chan.disable();
    // acc_z_chan.disable();
    // gyr_x_chan.disable();
    // gyr_y_chan.disable();
    // gyr_z_chan.disable();

    let acc_scale = acc_x_chan.data_format().scale();
    let gyr_scale = gyr_x_chan.data_format().scale();

    dev.set_num_kernel_buffers(8).unwrap();

    let mut buffer = dev
        .create_buffer(1, false)
        .map_err(|_| {
            println!("failed to create buffer");
            dev.remove_trigger().unwrap();
        })
        .unwrap();

    // buffer.set_blocking_mode(true).unwrap();

    while running.load(Ordering::SeqCst) {
        let _data_size = buffer.refill();
        // print!("data ready...");

        let ts = buffer.channel_iter::<u64>(&ts_chan);
        let (ax, ay, az) = (
            buffer.channel_iter::<i16>(&acc_x_chan),
            buffer.channel_iter::<i16>(&acc_y_chan),
            buffer.channel_iter::<i16>(&acc_z_chan),
        );

        let acc: Vec<_> = itertools::izip!(ax, ay, az)
            .map(move |(x, y, z)| {
                [
                    *x as f64 * acc_scale,
                    *y as f64 * acc_scale,
                    *z as f64 * acc_scale,
                ]
            })
            .collect();
        // debug reserve
        // acc.iter().for_each(|x| println!("{:?}", x));

        let (gx, gy, gz) = (
            buffer.channel_iter::<i16>(&gyr_x_chan),
            buffer.channel_iter::<i16>(&gyr_y_chan),
            buffer.channel_iter::<i16>(&gyr_z_chan),
        );
        let gyr: Vec<_> = itertools::izip!(gx, gy, gz)
            .map(move |(x, y, z)| {
                [
                    *x as f64 * gyr_scale,
                    *y as f64 * gyr_scale,
                    *z as f64 * gyr_scale,
                ]
            })
            .collect();
        // debug reserve
        // gyr.iter().for_each(|x| println!("{:?}", x));

        let imu: Vec<_> = itertools::izip!(ts, acc.iter(), gyr.iter()).collect();
        for i in imu {
            let time = DateTime::<Utc>::from(SystemTime::UNIX_EPOCH + Duration::from_nanos(*i.0))
                .format("%T%.6f");
            // println!("time = {}", time);
            // println!("imu data = {:?}", i);
            println!("ts = {}, data = {:?} {:?}", time, i.1, i.2);
        }
    }

    // although buffer can drop automatically,
    // but if buffer still alive(enable), trigger cannot remove success
    // so we drop mannually here, or put buffer into a scoped area
    drop(buffer);
    dev.remove_trigger().unwrap();
}
