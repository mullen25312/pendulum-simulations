/*--------------------------------------------------------------------------------------------------------------
Jahn, Benjamin aka mullen25312
comments:
leapfrog ode solver: one step to be energy invariant (semi implicite euler method, simplestic solver)
 *-------------------------------------------------------------------------------------------------------------*/
#![allow(unused_variables)] // matrix function pre-compute sines and cosines, some of them are not used
#![allow(non_snake_case)] // keep uppercase letters for matrices

// mod multi_link_pendulum_ode_8;
// const DOF: i64 = 8;
mod multi_link_pendulum_ode_32;
const DOF: i64 = 32;

use std::time::{Instant, Duration};
use ndarray::prelude::*;
use csv;


//  use rand::Rng;
use plotly::{Plot, Scatter};

fn simplectic_euler_vec(f:&dyn Fn(f64, Array1::<f64>) -> Array1::<f64>, x0:Array1::<f64>, ts:f64, k:i64) -> (Vec<f64>, Vec<Array1::<f64>>) {
    let n:usize = x0.dim();
    let doq:usize = x0.dim() / 2;


    // prepare solution vectors
    let mut x:Vec<Array1::<f64>> = vec![Array1::zeros(n); k as usize +1]; x[0] = x0;
    let mut t:Vec<f64> = vec![0.0; k as usize +1];

    let mut q :Vec<Array1::<f64>> = vec![Array1::zeros(doq); k as usize +1];
    let mut dq :Vec<Array1::<f64>> = vec![Array1::zeros(doq); k as usize +1];

    for idx in 1..doq {
        q[0][idx]= x[0][idx];
        dq[0][idx]= x[0][idx+doq];

    }

    // run euler steps
    for idx in 1..k+1 {
        t[idx as usize] = t[idx as usize-1] + ts;
        x[idx as usize] = x[idx as usize-1].clone() + ts*f(t[idx as usize-1], x[idx as usize-1].clone());


        for idx2 in 1..doq {
            dq[idx as usize][idx2] = x[idx as usize][idx2+doq];
        }

        q[idx as usize] = q[idx as usize-1].clone() + dq[idx as usize].clone() * ts;

        for idx2 in 1..doq {
            x[idx as usize][idx2] = q[idx as usize][idx2];
        } 
    }
    
    return (t, x)
}

fn main() {
    println!("########## Robotic Equation Solver ##########");

    // simulation settings
    let ts: f64 = 0.0001; // 0.1ms sample time 
    let k: i64 = 10000*24; // number of samples

    // setup signal vectors
    let t: Vec<f64>; // = vec![0.0; k as usize +1];
    let x_vec: Vec<Array1::<f64>>;

    // run simulation
    let mut x0_vec: Array1::<f64> = Array::from_vec(vec![0.0; 2*DOF as usize]);
    x0_vec[2*(DOF as usize)-1] = 0.2;

    let start: Instant = Instant::now();
    // (t, x_vec) = simplectic_euler_vec(&multi_link_pendulum_ode_8::robotic_system, x0_vec, ts, k as i64);
    (t, x_vec) = simplectic_euler_vec(&multi_link_pendulum_ode_32::robotic_system, x0_vec, ts, k as i64);
    let duration: Duration = start.elapsed();
    println!("Time elapsed for numerical integration is: {:?}", duration);

    // visualize simulation
    let mut plot: Plot = Plot::new();
    (0..DOF).for_each(|idx:i64| plot.add_trace(Scatter::new(t.clone(), x_vec.iter().map(|x:&Array1::<f64>| x[idx as usize]).collect())));
    // plot.write_image("pendulum", ImageFormat::PNG, 1200, 500, 1.0);

    // export data to csv
    let mut wtr = csv::Writer::from_path("./multi_link_pendulum_solution.csv").unwrap();

    const EMPTY_STRING: String = String::new();

    for idx in 0..k/100 {
        let mut tmp:[String;(DOF+1) as usize] = [EMPTY_STRING; (DOF+1) as usize]; 
        tmp[0] = t[(idx*100) as usize].to_string();
        for idx2 in 0..DOF {
            tmp[(idx2+1) as usize] = x_vec[(idx * 100) as usize][idx2 as usize].to_string();
        }
        let _ = wtr.write_record(&tmp);
    }

    let _ = wtr.flush();



}