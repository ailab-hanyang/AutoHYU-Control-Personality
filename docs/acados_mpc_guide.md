# Acados MPC Implementation Guide

## Table of Contents
1. [System Dynamics Definition](#1-system-dynamics-definition)
2. [Model Parameters](#2-model-parameters)
3. [Constraints](#3-constraints)
4. [Cost Function](#4-cost-function)
5. [Solver Settings](#5-solver-settings)
6. [Implementation Checklist](#6-implementation-checklist)
7. [Solver Initialization and Execution](#7-solver-initialization-and-execution)
8. [Memory Management](#8-memory-management)
9. [Real-time Implementation Tips](#9-real-time-implementation-tips)
10. [Common Issues and Solutions](#10-common-issues-and-solutions)
11. [Advanced Topics](#11-advanced-topics)
12. [ROS Integration Tips](#12-ros-integration-tips)
13. [Debug and Visualization](#13-debug-and-visualization)
14. [Performance Optimization](#14-performance-optimization)
15. [Safety Considerations](#15-safety-considerations)
16. [Installation and Environment Setup](#16-installation-and-environment-setup)
17. [Solver Selection Guide](#17-solver-selection-guide)
18. [Problem Size Optimization](#18-problem-size-optimization)
19. [Advanced Numerical Considerations](#19-advanced-numerical-considerations)

## 1. System Dynamics Definition

### Basic Structure
```python
def ode_model() -> AcadosModel:
    # State and input definition
    x = SX.sym('x', nx)    # State vector
    u = SX.sym('u', nu)    # Input vector
    p = SX.sym('p', np)    # Parameters (optional)
    
    # System dynamics
    dx = vertcat(
        f1(x, u, p),    # State 1 dynamics
        f2(x, u, p),    # State 2 dynamics
        ...
    )
    
    # Create Acados model
    model = AcadosModel()
    model.f_expl_expr = dx
    model.x = x
    model.u = u
    model.p = p    # Optional
    model.name = 'model_name'
    
    return model
```

### Example: Kinematic Bicycle Model
```python
def ode_model() -> AcadosModel:
    # States: [x, y, psi, v]
    x = SX.sym('x')
    y = SX.sym('y')
    psi = SX.sym('psi')
    v = SX.sym('v')
    state = vertcat(x, y, psi, v)
    
    # Inputs: [delta, a]
    delta = SX.sym('delta')
    a = SX.sym('a')
    input = vertcat(delta, a)
    
    # Dynamics
    dx = v * cos(psi)
    dy = v * sin(psi)
    dpsi = v * tan(delta) / L  # L: wheelbase
    dv = a
    
    dynamics = vertcat(dx, dy, dpsi, dv)
    
    # Create model
    model = AcadosModel()
    model.f_expl_expr = dynamics
    model.x = state
    model.u = input
    
    return model
```

## 2. Model Parameters

### Parameter Definition
```python
# Define parameters
param1 = SX.sym('param1')
param2 = SX.sym('param2')
params = vertcat(param1, param2)

# Register parameters
ocp.model.p = params
ocp.parameter_values = np.zeros(params.size()[0])
```

### Runtime Parameter Update
```python
# Update parameters during runtime
acados_solver.set('p', new_parameter_values)
```

## 3. Constraints

### Constraint Dimensions
```python
# State dimensions
nx = model.x.size()[0]  # 상태 벡터 차원
nu = model.u.size()[0]  # 입력 벡터 차원

# Number of shooting nodes
N = ocp.solver_options.N_horizon  # 예측 구간의 노드 수

# Constraint dimensions per node
nbx = 3  # 상태 제약조건 수
nbu = 2  # 입력 제약조건 수
nh = 5   # 비선형 제약조건 수

# Set dimensions
ocp.dims.nbx = nbx   # 상태 제약조건 수
ocp.dims.nbu = nbu   # 입력 제약조건 수
ocp.dims.nh = nh     # 비선형 제약조건 수
ocp.dims.ns = 2      # 슬랙 변수 수 (소프트 제약조건용)

# Terminal constraints can have different dimensions
ocp.dims.nbx_e = nbx_e  # 종단 상태 제약조건 수
ocp.dims.nh_e = nh_e    # 종단 비선형 제약조건 수
ocp.dims.ns_e = 2       # 종단 슬랙 변수 수
```

### Hard Constraints

#### State Constraints
```python
# Box constraints on states
ocp.constraints.lbx = np.array([x_min, y_min, ...])  # nbx 차원
ocp.constraints.ubx = np.array([x_max, y_max, ...])  # nbx 차원
ocp.constraints.idxbx = np.array([0, 1, ...])        # nbx 차원

# Terminal state constraints (can be different)
ocp.constraints.lbx_e = np.array([x_min_e, y_min_e, ...])  # nbx_e 차원
ocp.constraints.ubx_e = np.array([x_max_e, y_max_e, ...])  # nbx_e 차원
ocp.constraints.idxbx_e = np.array([0, 1, ...])            # nbx_e 차원
```

#### Input Constraints
```python
# Box constraints on inputs
ocp.constraints.lbu = np.array([u1_min, u2_min])  # nbu 차원
ocp.constraints.ubu = np.array([u1_max, u2_max])  # nbu 차원
ocp.constraints.idxbu = np.array([0, 1])          # nbu 차원
```

#### Nonlinear Constraints
```python
# Define nonlinear constraints
nl_constraint = vertcat(
    constraint1(x, u),
    constraint2(x, u),
    ...  # nh 개의 제약조건
)

ocp.model.con_h_expr = nl_constraint               # nh 차원
ocp.constraints.lh = np.array([h1_min, h2_min])   # nh 차원
ocp.constraints.uh = np.array([h1_max, h2_max])   # nh 차원

# Terminal nonlinear constraints (can be different)
nl_constraint_e = vertcat(
    constraint1_e(x),
    constraint2_e(x),
    ...  # nh_e 개의 제약조건
)

ocp.model.con_h_expr_e = nl_constraint_e          # nh_e 차원
ocp.constraints.lh_e = np.array([h1_min_e, ...])  # nh_e 차원
ocp.constraints.uh_e = np.array([h1_max_e, ...])  # nh_e 차원
```

### Soft Constraints
```python
# Slack variables for soft constraints
ocp.constraints.Zl = np.array([1.0, 1.0])   # ns 차원 (quadratic penalty)
ocp.constraints.Zu = np.array([1.0, 1.0])   # ns 차원 (quadratic penalty)
ocp.constraints.zl = np.array([0.1, 0.1])   # ns 차원 (linear penalty)
ocp.constraints.zu = np.array([0.1, 0.1])   # ns 차원 (linear penalty)

# Terminal slack variables (if different)
ocp.constraints.Zl_e = np.array([1.0, 1.0])  # ns_e 차원
ocp.constraints.Zu_e = np.array([1.0, 1.0])  # ns_e 차원
ocp.constraints.zl_e = np.array([0.1, 0.1])  # ns_e 차원
ocp.constraints.zu_e = np.array([0.1, 0.1])  # ns_e 차원

# Slack variables to specific constraints
ocp.constraints.idxsh = np.array([0, 1])    # 소프트 제약조건으로 만들 비선형 제약조건의 인덱스
ocp.constraints.idxsh_e = np.array([0, 1])  # 종단 시점의 소프트 제약조건 인덱스
```

## 4. Cost Function

### External Cost Function
```python
# State and input costs
cost_x = vertcat(x[0] - x_ref, x[1] - y_ref)
cost_u = vertcat(u[0], u[1])

# Quadratic costs
Q = np.diag([q1, q2])  # State weights
R = np.diag([r1, r2])  # Input weights

# Define cost function
ocp.model.cost_expr_ext_cost = (
    0.5 * cost_x.T @ Q @ cost_x +
    0.5 * cost_u.T @ R @ cost_u
)

# Terminal cost
ocp.model.cost_expr_ext_cost_e = 0.5 * cost_x.T @ Q @ cost_x
```

## 5. Solver Settings

### Basic Settings
```python
# Horizon settings
ocp.solver_options.tf = 1.0  # Time horizon
ocp.solver_options.N_horizon = 40  # Number of shooting nodes

# QP solver
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.hpipm_mode = 'SPEED'
ocp.solver_options.qp_solver_iter_max = 100

# Integrator
ocp.solver_options.integrator_type = 'ERK'
ocp.solver_options.sim_method_num_stages = 4  # RK4
ocp.solver_options.sim_method_num_steps = 3

# NLP solver
ocp.solver_options.nlp_solver_type = 'SQP_RTI'
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
```

## 6. Implementation Checklist

### Initialization
- [ ] Check state dimensions
- [ ] Check input dimensions
- [ ] Verify parameter dimensions
- [ ] Initialize solver with feasible initial state

### Numerical Stability
- [ ] Scale states and inputs appropriately
- [ ] Check weight magnitudes
- [ ] Verify constraint bounds are reasonable

### Runtime Considerations
- [ ] Implement warm starting
- [ ] Handle infeasible solutions
- [ ] Monitor solver status
- [ ] Check computation time

### Memory Management
- [ ] Free solver memory when done
- [ ] Monitor memory usage
- [ ] Clean up temporary files

### Error Handling
- [ ] Implement solution status checking
- [ ] Handle numerical errors
- [ ] Implement fallback controllers

### Real-time Implementation
- [ ] Use RTI scheme for real-time applications
- [ ] Implement timing checks
- [ ] Monitor real-time performance 

## 7. Solver Initialization and Execution

### Solver Creation
```python
# Create solver
acados_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

# Build mode options
acados_solver = AcadosOcpSolver(ocp, 
    generate=True,  # C 코드 생성
    build=True,     # 생성된 코드 빌드
    json_file='acados_ocp.json'
)
```

### Initial State and Warm Start
```python
# Set initial state
acados_solver.set(0, "lbx", x0)
acados_solver.set(0, "ubx", x0)

# Warm start from previous solution
for i in range(N):
    acados_solver.set(i, "x", x_traj[i])
    acados_solver.set(i, "u", u_traj[i])
```

### Solver Execution
```python
# Single step solve
status = acados_solver.solve()

# Get solution
for i in range(N):
    x_sol = acados_solver.get(i, "x")
    u_sol = acados_solver.get(i, "u")

# Get solver statistics
solve_time = acados_solver.get_stats("time_tot")
sqp_iter = acados_solver.get_stats("sqp_iter")
```

### Error Handling
```python
# Status codes
STATUS_SUCCESS = 0
STATUS_MAX_ITER = 1
STATUS_INFEASIBLE = 2
...

def check_acados_status(status):
    if status != STATUS_SUCCESS:
        if status == STATUS_MAX_ITER:
            print("Warning: Maximum iterations reached")
        elif status == STATUS_INFEASIBLE:
            print("Error: Problem infeasible")
            # 대체 제어 전략 실행
        return False
    return True
```

## 8. Memory Management

### Resource Cleanup
```python
# Clean up solver
del acados_solver

# Remove generated files
os.system('rm -r c_generated_code')
```

### Memory Usage Optimization
```python
# Pre-allocate arrays
x_traj = np.zeros((N+1, nx))
u_traj = np.zeros((N, nu))

# Reuse arrays instead of creating new ones
for i in range(N):
    acados_solver.get(i, "x", x_traj[i])
    acados_solver.get(i, "u", u_traj[i])
```

## 9. Real-time Implementation Tips

### Timing Management
```python
# Set timing constraints
ocp.solver_options.time_limit = 0.001  # 1ms time limit

# Monitor execution time
start_time = time.time()
status = acados_solver.solve()
solve_time = time.time() - start_time

if solve_time > 0.001:
    print(f"Warning: Solver exceeded time limit: {solve_time}s")
```

### Solution Quality vs Speed
```python
# Fast but approximate solution
ocp.solver_options.nlp_solver_type = 'SQP_RTI'
ocp.solver_options.qp_solver_iter_max = 50
ocp.solver_options.hpipm_mode = 'SPEED'

# More accurate but slower solution
ocp.solver_options.nlp_solver_type = 'SQP'
ocp.solver_options.qp_solver_iter_max = 100
ocp.solver_options.hpipm_mode = 'ROBUST'
```

### Fallback Strategies
```python
def solve_with_fallback():
    # Try optimal solution first
    status = acados_solver.solve()
    
    if status != 0:
        # Fallback 1: Increase iterations
        ocp.solver_options.qp_solver_iter_max += 50
        status = acados_solver.solve()
        
        if status != 0:
            # Fallback 2: Use previous solution
            u_applied = u_prev
            
            # Fallback 3: Emergency controller
            u_applied = emergency_controller()
    
    return u_applied
```

## 10. Common Issues and Solutions

### Numerical Issues
- Scale states and inputs to similar magnitudes
- Use appropriate tolerances
- Check for singular matrices in constraints
- Monitor condition number of Hessian

### Infeasibility Handling
- Gradually soften constraints
- Check initial guess quality
- Verify constraint consistency
- Use constraint relaxation

### Performance Issues
- Profile solver execution
- Optimize problem size
- Use appropriate solver settings
- Consider problem conditioning

### Code Generation Issues
- Check compiler compatibility
- Verify library paths
- Monitor generated code size
- Clean old generated files 

## 11. Advanced Topics

### Multiple Shooting vs Single Shooting
```python
# Multiple shooting (기본값)
ocp.solver_options.shooting_mode = 'MULTIPLE'

# Single shooting
ocp.solver_options.shooting_mode = 'SINGLE'
```
- Multiple shooting: 더 안정적이지만 계산 비용이 높음
- Single shooting: 빠르지만 수치적으로 불안정할 수 있음

### Discretization Methods
```python
# Explicit Runge-Kutta 4
ocp.solver_options.integrator_type = 'ERK'
ocp.solver_options.sim_method_num_stages = 4

# Implicit Runge-Kutta (stiff systems)
ocp.solver_options.integrator_type = 'IRK'
```

### Condensing Methods
```python
# Partial condensing (메모리 효율적)
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.hpipm_mode = 'SPEED'

# Full condensing (작은 문제에 적합)
ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
```

### Sensitivity Computation
```python
# Exact Hessian (정확하지만 느림)
ocp.solver_options.hessian_approx = 'EXACT'

# Gauss-Newton (빠르지만 근사)
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
```

## 12. ROS Integration Tips

### Message Handling
```python
def state_callback(msg):
    # Convert ROS message to Acados format
    x0 = np.array([
        msg.pose.position.x,
        msg.pose.position.y,
        msg.twist.linear.x
    ])
    
    # Update initial state
    acados_solver.set(0, "lbx", x0)
    acados_solver.set(0, "ubx", x0)
```

### Timing Synchronization
```python
def control_loop():
    rate = rospy.Rate(100)  # 100Hz
    while not rospy.is_shutdown():
        try:
            status = acados_solver.solve()
            if status == 0:
                u = acados_solver.get(0, "u")
                publish_control(u)
        except:
            emergency_stop()
        rate.sleep()
```

## 13. Debug and Visualization

### Trajectory Visualization
```python
def visualize_prediction():
    marker_array = MarkerArray()
    for i in range(N):
        x = acados_solver.get(i, "x")
        marker = create_marker(x)
        marker_array.markers.append(marker)
    pub_prediction.publish(marker_array)
```

### Solver Diagnostics
```python
def print_diagnostics():
    print(f"Solve time: {acados_solver.get_stats('time_tot')}")
    print(f"SQP iterations: {acados_solver.get_stats('sqp_iter')}")
    print(f"QP status: {acados_solver.get_stats('qp_status')}")
    print(f"Cost value: {acados_solver.get_cost()}")
```

## 14. Performance Optimization

### Memory Pre-allocation
```python
class AcadosMPC:
    def __init__(self):
        # Pre-allocate buffers
        self.x_traj = np.zeros((N+1, nx))
        self.u_traj = np.zeros((N, nu))
        self.p = np.zeros(np)
        
    def solve(self):
        # Use pre-allocated memory
        for i in range(N):
            self.acados_solver.get(i, "x", self.x_traj[i])
```

### Parallel Computation
```python
# Enable parallel computations
ocp.solver_options.parallel_computations = True
```

### Code Generation Optimization
```python
# Optimize generated code
ocp.code_export_directory = 'c_generated_code'
ocp.solver_options.generate_hess = True
ocp.solver_options.codegen_options = {
    'optimize': True,
    'cleanup': True
}
```

## 15. Safety Considerations

### Constraint Handling
```python
def ensure_safety():
    # Gradually relax constraints if infeasible
    while status != 0 and safety_margin > min_safety:
        safety_margin *= 0.9
        update_safety_constraints(safety_margin)
        status = acados_solver.solve()
```

### Fallback Controller
```python
class SafetyController:
    def __init__(self):
        self.last_safe_input = None
        self.emergency_controller = PIDController()
    
    def get_safe_input(self):
        if not self.is_solution_safe():
            if self.last_safe_input is not None:
                return self.last_safe_input
            return self.emergency_controller.compute()
```

### Solution Validation
```python
def validate_solution():
    # Check physical limits
    x_pred = acados_solver.get(0, "x")
    u_pred = acados_solver.get(0, "u")
    
    if not is_state_feasible(x_pred) or not is_input_feasible(u_pred):
        return False
    
    # Check solution quality
    if acados_solver.get_stats('qp_iter') >= max_qp_iter:
        return False
    
    return True
```

## 16. Installation and Environment Setup

### Building from Source
```bash
# Dependencies
sudo apt-get install gcc cmake pkg-config git

# Clone and build
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=ON ..
make -j4
make install
```

### Environment Variables
```bash
# Add to ~/.bashrc
export ACADOS_SOURCE_DIR="/path/to/acados"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_SOURCE_DIR/lib
export PYTHONPATH=$PYTHONPATH:$ACADOS_SOURCE_DIR/interfaces/acados_template
```

### Python Interface Setup
```bash
cd interfaces/acados_template
pip3 install -e .
```

## 17. Solver Selection Guide

### Problem Size Considerations
```python
# Small-scale problems (nx + nu < 50, N < 20)
ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'

# Medium-scale problems
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'

# Large-scale problems (nx + nu > 100, N > 50)
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_OSQP'
```

### Solver Comparison
| Solver | Pros | Cons | Best Use Case |
|--------|------|------|---------------|
| HPIPM | 빠름, 메모리 효율적 | 민감한 파라미터 튜닝 | 일반적인 MPC |
| qpOASES | 안정적, 정확함 | 큰 문제에 부적합 | 작은 문제 |
| OSQP | 견고함, 큰 문제 가능 | 상대적으로 느림 | 큰 문제, 견고성 필요 |

### Performance Benchmarking
```python
def benchmark_solvers():
    solvers = ['FULL_CONDENSING_QPOASES', 
               'PARTIAL_CONDENSING_HPIPM',
               'PARTIAL_CONDENSING_OSQP']
    
    results = {}
    for solver in solvers:
        ocp.solver_options.qp_solver = solver
        acados_solver = AcadosOcpSolver(ocp, build=True)
        
        times = []
        for _ in range(100):
            status = acados_solver.solve()
            times.append(acados_solver.get_stats('time_tot'))
        
        results[solver] = {
            'mean_time': np.mean(times),
            'max_time': np.max(times),
            'min_time': np.min(times)
        }
```

## 18. Problem Size Optimization

### State/Input Reduction
```python
# Original large state
x_full = vertcat(x, y, v, a, jerk, theta, omega, ...)

# Reduced state for control
x_reduced = vertcat(x, y, v, theta)

# State transformation matrices
T_reduce = np.array([...])  # Full -> Reduced
T_expand = np.array([...])  # Reduced -> Full
```

### Horizon Length Selection
```python
def optimize_horizon_length():
    # Trade-off between prediction quality and computation time
    N_candidates = [20, 30, 40, 50]
    
    for N in N_candidates:
        ocp.solver_options.N_horizon = N
        # Run benchmark tests
        # Measure prediction quality
        # Measure computation time
```

### Memory Optimization
```python
# Minimize memory allocation
class EfficientMPC:
    def __init__(self):
        # Pre-allocate all possible buffers
        self.x_traj = np.zeros((N_max+1, nx))
        self.u_traj = np.zeros((N_max, nu))
        self.constraints = np.zeros(nh)
        
    def solve_efficient(self):
        # Use pre-allocated memory
        self.acados_solver.solve()
        self.acados_solver.get(0, "x", self.x_traj[0])
```

## 19. Advanced Numerical Considerations

### Scaling and Conditioning
```python
class ProblemScaling:
    def __init__(self):
        # Define scaling factors
        self.x_scale = np.array([10.0, 10.0, 5.0, 1.0])
        self.u_scale = np.array([1.0, 0.5])
        
    def scale_problem(self):
        # Scale states and inputs
        x_scaled = x / self.x_scale
        u_scaled = u / self.u_scale
        
        # Scale constraints
        lbx_scaled = lbx / self.x_scale
        ubx_scaled = ubx / self.x_scale
```

### Numerical Stability
```python
def improve_conditioning():
    # Add regularization
    eps = 1e-6
    Q_reg = Q + eps * np.eye(nx)
    R_reg = R + eps * np.eye(nu)
    
    # Normalize dynamics
    dx_norm = dx / np.maximum(abs(dx), 1.0)
```

### Constraint Softening Strategies
```python
class ConstraintSoftening:
    def __init__(self):
        self.slack_weights = np.logspace(-3, 3, 7)
        
    def progressive_softening(self):
        for weight in self.slack_weights:
            ocp.constraints.Zl = weight * np.ones(ns)
            ocp.constraints.Zu = weight * np.ones(ns)
            status = acados_solver.solve()
            if status == 0:
                break
``` 