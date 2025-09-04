use ndarray::*;
use ndarray_linalg::*;

const DOF:usize = 2;

fn M_flat(q: [f64; 2]) ->  [f64; 4] {
    let q_0 = q[0]; let q_0s = q_0.sin(); let q_0c = q_0.cos(); 
    let q_1 = q[1]; let q_1s = q_1.sin(); let q_1c = q_1.cos(); 
    let out1 = [0.1875*q_0s.powi(2) + 0.1875*q_0c.powi(2), 0.04375*q_0s*q_1s + 0.04375*q_0c*q_1c, 0.04375*q_0s*q_1s + 0.04375*q_0c*q_1c, 0.0153125*q_1s.powi(2) + 0.0153125*q_1c.powi(2)];
    out1
}
fn C_flat(q: [f64; 2], dq: [f64; 2]) ->  [f64; 4] {
    let q_0 = q[0]; let dq_0 = dq[0]; let q_0s = q_0.sin(); let q_0c = q_0.cos(); 
    let q_1 = q[1]; let dq_1 = dq[1]; let q_1s = q_1.sin(); let q_1c = q_1.cos(); 
    let out1 = [0.0 ,0.04375*dq_1*q_0s*q_1c - 0.04375*dq_1*q_1s*q_0c, -0.04375*dq_0*q_0s*q_1c + 0.04375*dq_0*q_1s*q_0c, 0.0];
    out1
}
fn D_flat() -> [f64; 4] {
    let out1 = [0.01, -0.005, -0.005, 0.005];
    out1
}
fn g_flat(q: [f64; 2]) ->  [f64; 2] {
    let q_0 = q[0]; let q_0s = q_0.sin(); let q_0c = q_0.cos(); 
    let q_1 = q[1]; let q_1s = q_1.sin(); let q_1c = q_1.cos(); 
    let out1 = [-4.905*q_0s, -0.858375*q_1s];
    out1
}

pub fn robotic_system(t:f64, x:Array1::<f64>) -> Array1::<f64> {
let mut q_vec = [0.0; DOF];
let mut dq_vec = [0.0; DOF];
for idx in 0..DOF {
q_vec[idx] = x[idx];
dq_vec[idx] = x[idx+DOF];
}
let q:Array1::<f64> = Array::from_vec(q_vec.to_vec());
let dq:Array1::<f64> = Array::from_vec(dq_vec.to_vec());
let N:usize = q.dim();
let dq_:Array1::<f64> = dq.clone();
let M: Array2::<f64> = Array::from_shape_vec((N,N), M_flat(q_vec).to_vec()).unwrap();
let C: Array2::<f64> = Array::from_shape_vec((N,N), C_flat(q_vec, dq_vec).to_vec()).unwrap();
let D: Array2::<f64> = Array::from_shape_vec((N,N), D_flat().to_vec()).unwrap();
let g: Array1::<f64> = Array::from_vec(g_flat(q_vec).to_vec());
let ddq_:Array1::<f64> = M.solve_into(-C.dot(&dq) - D.dot(&dq) - g).unwrap();
return concatenate(Axis(0), &[dq_.view(), ddq_.view()]).unwrap();
}