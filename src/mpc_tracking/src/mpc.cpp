#include "mpc_tracking/mpc.h"

Mpc::Mpc() {
    
    
    N_ = 10;
    dt_ = 0.1;
    u_max_ = 4;
    w_max_ = 2;
    vector<double> weights = {10,10,1,1,1}; //Q,R
    u_min_ = - u_max_;
    w_min_ = - w_max_;
    
    Q_ = DM::zeros(3,3); //索引之前初始化size
    R_ = DM::zeros(2,2);
    

    setWeights(weights);
    kinematic_equation_ = setKinematicEquation();
}
Mpc::~Mpc() {}

void Mpc::setWeights(vector<double> weights) {
    //cout << "setweights" << endl;
    Q_(0, 0) = weights[0];
    Q_(1, 1) = weights[1];
    Q_(2, 2) = weights[2];
    R_(0, 0) = weights[3];
    R_(1, 1) = weights[4];
    //R_(2, 2) = weights[5];
    //cout << "set weight finish" << endl;

}

Function Mpc::setKinematicEquation() {
    //cout << "set kinematic" << endl;
    MX x = MX::sym("x");
    MX y = MX::sym("y");
    MX theta = MX::sym("theta");
    MX state_vars = MX::vertcat({x, y, theta});

    MX v = MX::sym("v");
    MX w = MX::sym("w");
    MX control_vars = MX::vertcat({v, w});
    
    //rhs means right hand side
    MX rhs = MX::vertcat({v * MX::cos(theta), v * MX::sin(theta), w});
    return Function("kinematic_equation", {state_vars, control_vars}, {rhs});
    // MX x = MX::sym("x");
    // MX y = MX::sym("y");
    // MX theta = MX::sym("theta");
    // MX state_vars = MX::vertcat({x, y, theta});

    // MX u = MX::sym("u");
    // MX v = MX::sym("v");
    // MX w = MX::sym("w");
    // MX control_vars = MX::vertcat({u, v, w});

    // MX rhs = u * MX::cos(theta) - v * MX::sin(theta);
    // rhs = MX::vertcat({rhs, u * MX::sin(theta) + v * MX::cos(theta), w});
    // return Function("kinematic_equation", {state_vars, control_vars}, {rhs});
}


bool Mpc::solve(Eigen::Vector3d current_states, Eigen::MatrixXd desired_states) {
    
    //cout << "jinqiujiele" << endl;
    Opti opti = Opti();

    Slice all;

    MX cost = 0;
    X = opti.variable(3, N_ + 1);
    U = opti.variable(2, N_);
    MX x = X(0, all);
    MX y = X(1, all);
    MX theta = X(2, all);
    MX v = U(0, all);
    MX w = U(1, all);
    //MX w = U(2, all);


    MX X_ref = opti.parameter(3, N_ + 1);
    MX X_cur = opti.parameter(3);
    DM x_tmp1 = {current_states[0], current_states[1], current_states[2]};

    opti.set_value(X_cur, x_tmp1);  //set current state
    cout << "set current state success" << endl;

    
    //按列索引
    vector<double> X_ref_v(desired_states.data(), desired_states.data() + desired_states.size());
    //auto tp1 = std::chrono::steady_clock::now();
    DM X_ref_d(X_ref_v);
    
    //X_ref_d.resize(3, N_ + 1);
    
    //cout << "desired_states:" << desired_states << endl;
    //cout << "X_ref_v:" << X_ref_v << endl;
    //cout << "X_ref_d:" << X_ref_d << endl;
    // DM x_tmp2(3, N_ + 1);
    // for (int i = 0; i < 3; ++i) {
    //     for (int j = 0; j <= N_; ++j) {
    //         x_tmp2(i, j) = desired_states(i, j);
    //     }
    // }
    X_ref = MX::reshape(X_ref_d, 3, N_ + 1);
    
    
    
    //opti.set_value(X_ref, X_ref_d);  //set reference traj

    // auto tp2 = std::chrono::steady_clock::now();
    // cout <<"set trajectory time:" 
    // << chrono::duration_cast<chrono::microseconds>(tp2 - tp1).count() 
    // << "microseconds" << endl;
    //cout << "set reference traj success" << endl;
    //cout << "x_ref:" << X_ref.size() << endl;

    //set costfunction
    for (int i = 0; i < N_; ++i) {
        MX X_err = X(all, i) - X_ref(all, i); 
        MX U_0 = U(all, i);
        //cout << "U_0 size:" << U_0.size() << endl;
        //cout << "cost size:" << cost_.size() << endl;
        cost += MX::mtimes({X_err.T(), Q_, X_err});
        //cout << "cost size:" << cost_.size() << endl; 
        cost += MX::mtimes({U_0.T(), R_, U_0});
        //cout << "cost size:" << cost_.size() << endl;
    }
    //cout << "cost size:" << cost_.size() << endl;
    cost += MX::mtimes({(X(all, N_) - X_ref(all, N_)).T(), Q_,
                        X(all, N_) - X_ref(all, N_)});
    //cout << "cost:" << cost << endl;
    opti.minimize(cost);
    //cout << "set cost success" << endl;

    //kinematic constrains
    for (int i = 0; i < N_; ++i) {
        vector<MX> input(2);
        input[0] = X(all, i);
        input[1] = U(all, i);
        MX X_next = kinematic_equation_(input)[0] * dt_ + X(all, i);
        opti.subject_to(X_next == X(all, i + 1));
    }

    //init value
    opti.subject_to(X(all, 0) == X_cur);

    //speed angle_speed limit
    

    opti.subject_to(0 <= v <= u_max_);
    opti.subject_to(w_min_ <= w <= w_max_);

    //set solver
    casadi::Dict solver_opts;
    solver_opts["expand"] = true; //MX change to SX for speed up
    solver_opts["ipopt.max_iter"] = 100;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = 0;
    solver_opts["ipopt.acceptable_tol"] = 1e-6;
    solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

    opti.solver("ipopt", solver_opts);

    //auto sol = opti.solve();
    solution_ = std::make_unique<casadi::OptiSol>(opti.solve());

    return true;
}
vector<double> Mpc::getFirstU() {
    vector<double> res;
    auto first_v =  solution_->value(U)(0, 0);
    auto first_w = solution_->value(U)(1, 0);
    
    //cout << "first_u" << first_u << " " << "first_v" << first_v << endl;

    res.push_back(static_cast<double>(first_v));
    res.push_back(static_cast<double>(first_w));
    return res;
}

vector<double> Mpc::getPredictX() {
    vector<double> res;
    auto predict_x = solution_->value(X);
    cout << "nomal" << endl;
    //cout << "predict_x size :" << predict_x.size() << endl;
    for (int i = 0; i <= N_; ++i) {
        res.push_back(static_cast<double>(predict_x(0, i)));
        res.push_back(static_cast<double>(predict_x(1, i)));
    }
    return res;
}
