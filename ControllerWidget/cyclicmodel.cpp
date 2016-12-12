#include "cyclicmodel.h"

CyclicModel::CyclicModel()
{
    m_BBfixed_CT   .resize(N_SAMPLES, EigenVector7d::Identity());
    m_BBfixed_Instr.resize(N_SAMPLES, EigenVector7d::Identity());
    m_BBfixed_BB   .resize(N_SAMPLES, EigenVector7d::Identity());
    m_Bird4        .resize(N_SAMPLES, EigenVector7d::Identity());
    m_timeData     .resize(N_SAMPLES, 0.0);

    m_BBfixed_CT_filtered.resize(N_SAMPLES, EigenVector7d::Zero());
    m_BBfixed_BB_filtered.resize(N_SAMPLES, EigenVector7d::Zero());
    m_Bird4_filtered     .resize(N_SAMPLES, EigenVector7d::Zero());

    m_BBfixed_CT_rectangular.resize(N_SAMPLES, EigenVector7d::Zero());
    m_BBfixed_BB_rectangular.resize(N_SAMPLES, EigenVector7d::Zero());
    m_Bird4_rectangular     .resize(N_SAMPLES, EigenVector7d::Zero());

    m_BBfixed_CT_polar.resize(N_SAMPLES, EigenVector7d::Zero());
    m_BBfixed_BB_polar.resize(N_SAMPLES, EigenVector7d::Zero());
    m_Bird4_polar     .resize(N_SAMPLES, EigenVector7d::Zero());

//    m_BBfixed_CT.pop_front();
//    m_BBfixed_CT.push_back(EigenAffineTransform3d::Identity());

    m_numSamples = 0;

    m_isTrained = false;
    m_lastTrainingTimestamp = 0.0;

    std::cout << "--- Initialized CyclicModel. ---" << std::endl;

    // SPUCE library - not used
//std::vector<double> firCoeff_b = spuce::design_window("hamming", FILTER_ORDER);
//std::vector<double> firCoeff_b = spuce::design_fir("maxflat", "LOW_PASS", 51, 1.0, 60.0);
//    spuce::fir<double> mFIR(50);
//    for(auto y : Y)
//        std::cout << y << std::endl;
}

CyclicModel::~CyclicModel()
{

}

void CyclicModel::operator =(const CyclicModel &Other)
{
    m_BBfixed_CT    = Other.m_BBfixed_CT;
    m_BBfixed_Instr = Other.m_BBfixed_Instr;
    m_BBfixed_BB    = Other.m_BBfixed_BB;
    m_Bird4         = Other.m_Bird4;

    m_BBfixed_CT_filtered = Other.m_BBfixed_CT_filtered;
    m_BBfixed_BB_filtered = Other.m_BBfixed_BB_filtered;
    m_Bird4_filtered      = Other.m_Bird4_filtered;

    m_BBfixed_CT_rectangular = Other.m_BBfixed_CT_rectangular;
    m_BBfixed_BB_rectangular = Other.m_BBfixed_BB_rectangular;
    m_Bird4_rectangular      = Other.m_Bird4_rectangular;

    m_BBfixed_CT_polar = Other.m_BBfixed_CT_polar;
    m_BBfixed_BB_polar = Other.m_BBfixed_BB_polar;
    m_Bird4_polar      = Other.m_Bird4_polar;

    // m_LowPassFilter = Other.m_LowPassFilter;

    m_timeData = Other.m_timeData;
    m_breathingSignal = Other.m_breathingSignal;

    m_numSamples = Other.m_numSamples;

    m_isTrained = Other.m_isTrained;
    m_lastTrainingTimestamp = Other.m_lastTrainingTimestamp;
}

void CyclicModel::addObservation(const EigenAffineTransform3d &T_BB_CT_curTipPos,
                                 const EigenAffineTransform3d &T_BB_targetPos,
                                 const EigenAffineTransform3d &T_Box_BBmobile,
                                 const EigenAffineTransform3d &T_BB_Box,
                                 const EigenAffineTransform3d &T_Bird4,
                                 const double sampleTime)
{
    // em_tip = m_BB_CT_curTipPos          // m_BB_CT_curTipPos is the current CT point w.r.t. BBfixed
    // em_instr = m_BB_targetPos           // m_BB_targetPos   is the current INST point w.r.t. BBfixed
    // local_Box_BBmobile = m_Box_BBmobile // m_Box_BBmobile    is the current BB point w.r.t. Box
    // local_BB_Box = m_BB_Box             // m_BB_Box         is the current Box point w.r.t. BBfixed
    // local_Bird4 = m_Bird4               // 4th EM tracker

    // pop oldest readings
    m_BBfixed_CT.pop_front();
    m_BBfixed_Instr.pop_front();
    m_BBfixed_BB.pop_front();
    m_Bird4.pop_front();
    m_timeData.pop_front();

    // local_BB_Box * local_Box_BBmobile = em_base; // T_BB_Box * T_Box_BBmobile = T_BBfixed_BBmobile
    // BBfixed_CT.insert(BBfixed_CT.end(), em_tip, em_tip+16);
    // BBfixed_Instr.insert(BBfixed_Instr.end(), em_instr, em_instr+16);
    // BBfixed_BB.insert(BBfixed_BB.end(), em_base, em_base+16);
    // Bird4.insert(Bird4.end(), local_Bird4, local_Bird4+16);

    EigenAffineTransform3d T_BB_BBm = T_BB_Box * T_Box_BBmobile;

    // axis angle
    Eigen::AngleAxisd tempEA_CT  (T_BB_CT_curTipPos.rotation()),
                      tempEA_Inst(T_BB_targetPos.rotation()),
                      tempEA_BB  (T_BB_BBm.rotation()),
                      tempEA_Bird(T_Bird4.rotation());

    EigenVector7d tempCT, tempInst, tempBB, tempBird4;

    tempCT << T_BB_CT_curTipPos(0,3), T_BB_CT_curTipPos(1,3), T_BB_CT_curTipPos(2,3),
              tempEA_CT.axis()(0), tempEA_CT.axis()(1), tempEA_CT.axis()(2), tempEA_CT.angle();

    tempInst << T_BB_targetPos(0,3), T_BB_targetPos(1,3), T_BB_targetPos(2,3),
              tempEA_Inst.axis()(0), tempEA_Inst.axis()(1), tempEA_Inst.axis()(2), tempEA_Inst.angle();

    tempBB << T_BB_BBm(0,3), T_BB_BBm(1,3), T_BB_BBm(2,3),
              tempEA_BB.axis()(0), tempEA_BB.axis()(1), tempEA_BB.axis()(2), tempEA_BB.angle();

    tempBird4 << T_Bird4(0,3), T_Bird4(1,3), T_Bird4(2,3),
              tempEA_Bird.axis()(0), tempEA_Bird.axis()(1), tempEA_Bird.axis()(2), tempEA_Bird.angle();

    // push newest readings
    m_BBfixed_CT.push_back(tempCT);
    m_BBfixed_Instr.push_back(tempInst);
    m_BBfixed_BB.push_back(tempBB);
    m_Bird4.push_back(tempBird4);
    m_timeData.push_back(sampleTime);

    m_numSamples++;
}

void CyclicModel::trainModel(const std::vector<double> data)
{
    if(m_numSamples < N_SAMPLES) // not enough samples
    {
        std::cerr << "m_numSamples is less than N_SAMPLES!" << std::endl;

        m_isTrained = false;
    }
    else if(m_isTrained) // already trained
    {
        std::cerr << "Already trained!" << std::endl;
    }
    else // train
    {
        size_t m = N_HARMONICS; // m = number of sinusoid components
        // double delta_t = SAMPLE_DELTA_TIME; // Time step for each collected data point
        size_t N_initpts = m_BBfixed_CT.size(); // number of initialization points
        size_t filterorder = FILTER_ORDER;
        size_t edge_effect = EDGE_EFFECT;
        double breath_expected = BREATH_RATE;
        double thresh = PEAK_THRESHOLD;

        if(N_initpts <= 2*edge_effect)
        {
            std::cerr << "Error: N_initpts < 2*edge_effect!" << std::endl;
            return;
        }

        // calculate things from inputs
        size_t num_states = m*2 + 2;
        size_t N_filtered = N_initpts - 2*edge_effect;

        // low pass filter the data

        m_LowPassFilter.run(m_BBfixed_CT, m_BBfixed_CT_filtered);
        //m_LowPassFilter.run(m_BBfixed_Instr, m_BBfixed_Instr_filtered);
        m_LowPassFilter.run(m_BBfixed_BB, m_BBfixed_BB_filtered);
        m_LowPassFilter.run(m_Bird4, m_Bird4_filtered);

        // TODO : peak detection
        // Use peak detection to look at the filtered data and find the period

        // % In vivo version has an error checker to make sure the breathing period is
        // % between 4 - 5.5 seconds

        // omega_0 = 2*pi*1/period; % 2*pi*breathing frequency (rad/sec)

        // Step 1. z = Ax + noise.  Assemble A and z.  Use rectangular fourier series.
        // use <<< cycle_recalculate >>> here

        m_isTrained = true;
    }
}

void CyclicModel::updatePeriod(const double shift)
{

}

double CyclicModel::getPrediction(const double timeShift, const std::vector<double> &x_polar, const std::vector<double> &x_rect)
{
    // Make sure the time coming in is already (t1[j] - t_begin)
    double x_des;

    if(m_isTrained)
    {
        x_des = 0.0;

        for (size_t i = 0; i < N_HARMONICS; i++)
        {
            x_des += x_polar[i+1] * sin( (i+1.0)*x_polar[N_HARMONICS+1]*timeShift + atan2( x_rect[i+N_HARMONICS+1], x_rect[i+1] ) );
        }
        x_des += x_polar[0];
    }
    else
    {
        x_des = std::numeric_limits<double>::quiet_NaN();
    }

    return x_des ;
}

void CyclicModel::retrainModel()
{

}

void CyclicModel::peakDetector()
{

}

std::shared_ptr< std::vector<double> > CyclicModel::cycle_recalculate(const std::vector<double> &inputs)
{
    //    std::ifstream is("inputs.txt");
    //    std::istream_iterator<double> start(is), end;
    //    std::vector<double> inputs(start, end);
    //    qDebug() << "Read " << inputs.size() << " numbers";

    //    std::shared_ptr< std::vector<double> > outputs = cycle_recalculate(inputs);

    //    std::ofstream output_file("outputs.txt");
    //    std::ostream_iterator<double> output_iterator(output_file, "\n");
    //    std::copy(outputs->begin(), outputs->end(), output_iterator);

//    QElapsedTimer elTimer;
//    elTimer.start();

    // TODO: add error checking to ensure that the vector size is correct

    std::vector<double>::const_iterator iter = inputs.begin();

    // get all of the constants
    int m = static_cast<int>(*iter); ++iter; // m = number of sinusoid components
    double delta_t = *iter; ++iter; // Time step for each collected data point
    int N_initpts = static_cast<int>(*iter); ++iter;  // number of initialization points
    int edge_effect = static_cast<int>(*iter); ++iter;
    double omega_0 = *iter; ++iter;

    // calculate things from inputs
    int num_states = m * 2 + 2;
    int N_filtered = N_initpts - 2 * edge_effect;

    // parse the already low pass filtered data
    Eigen::Map<Eigen::VectorXd> z_init_x(iter._Ptr, N_filtered); std::advance(iter, N_filtered); // Initialize with the previously lowpass filtered data
    Eigen::Map<Eigen::VectorXd> z_init_y(iter._Ptr, N_filtered); std::advance(iter, N_filtered);
    Eigen::Map<Eigen::VectorXd> z_init_z(iter._Ptr, N_filtered); std::advance(iter, N_filtered);
    Eigen::Map<Eigen::VectorXd> z_init_xaxis(iter._Ptr, N_filtered); std::advance(iter, N_filtered);
    Eigen::Map<Eigen::VectorXd> z_init_yaxis(iter._Ptr, N_filtered); std::advance(iter, N_filtered);
    Eigen::Map<Eigen::VectorXd> z_init_zaxis(iter._Ptr, N_filtered); std::advance(iter, N_filtered);
    Eigen::Map<Eigen::VectorXd> z_init_angle(iter._Ptr, N_filtered); std::advance(iter, N_filtered);

    // allocate zero matrices
    Eigen::MatrixXd A_init_x(N_filtered, num_states - 1); A_init_x.setZero(); A_init_x.col(0).setOnes();
    Eigen::MatrixXd A_init_y(N_filtered, num_states - 1); A_init_y.setZero(); A_init_y.col(0).setOnes();
    Eigen::MatrixXd A_init_z(N_filtered, num_states - 1); A_init_z.setZero(); A_init_z.col(0).setOnes();
    Eigen::MatrixXd A_init_xaxis(N_filtered, num_states - 1); A_init_xaxis.setZero(); A_init_xaxis.col(0).setOnes();
    Eigen::MatrixXd A_init_yaxis(N_filtered, num_states - 1); A_init_yaxis.setZero(); A_init_yaxis.col(0).setOnes();
    Eigen::MatrixXd A_init_zaxis(N_filtered, num_states - 1); A_init_zaxis.setZero(); A_init_zaxis.col(0).setOnes();
    Eigen::MatrixXd A_init_angle(N_filtered, num_states - 1); A_init_angle.setZero(); A_init_angle.col(0).setOnes();

    for (int i = 0; i < N_filtered; i++)
    {
        for (int j = 1; j <= m; j++)
        {
            A_init_x(i, j)			= sin(j*omega_0*i*delta_t);
            A_init_y(i, j)			= sin(j*omega_0*i*delta_t);
            A_init_z(i, j)			= sin(j*omega_0*i*delta_t);
            A_init_xaxis(i, j)		= sin(j*omega_0*i*delta_t);
            A_init_yaxis(i, j)		= sin(j*omega_0*i*delta_t);
            A_init_zaxis(i, j)		= sin(j*omega_0*i*delta_t);
            A_init_angle(i, j)		= sin(j*omega_0*i*delta_t);
            A_init_x(i, j + m)		= cos(j*omega_0*i*delta_t);
            A_init_y(i, j + m)		= cos(j*omega_0*i*delta_t);
            A_init_z(i, j + m)		= cos(j*omega_0*i*delta_t);
            A_init_xaxis(i, j + m)	= cos(j*omega_0*i*delta_t);
            A_init_yaxis(i, j + m)	= cos(j*omega_0*i*delta_t);
            A_init_zaxis(i, j + m)	= cos(j*omega_0*i*delta_t);
            A_init_angle(i, j + m)	= cos(j*omega_0*i*delta_t);
        }
    }

    // Step 2. Use least squares estimate to solve for x.
    //VectorXd x_init_x = pseudoInverse(A_init_x) * z_init_x;
    Eigen::VectorXd x_init_x = A_init_x.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_x);
    Eigen::VectorXd x_init_y = A_init_y.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_y);
    Eigen::VectorXd x_init_z = A_init_z.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_z);
    Eigen::VectorXd x_init_xaxis = A_init_xaxis.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_xaxis);
    Eigen::VectorXd x_init_yaxis = A_init_yaxis.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_yaxis);
    Eigen::VectorXd x_init_zaxis = A_init_zaxis.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_zaxis);
    Eigen::VectorXd x_init_angle = A_init_angle.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init_angle);

    //VectorXd x_rect_x = x_init_x;
    //VectorXd x_rect_y = x_init_y;
    //VectorXd x_rect_z = x_init_z;
    //VectorXd x_rect_xaxis = x_init_xaxis;
    //VectorXd x_rect_yaxis = x_init_yaxis;
    //VectorXd x_rect_zaxis = x_init_zaxis;
    //VectorXd x_rect_angle = x_init_angle;

    // Step 3. Convert from rectangular into polar and assemble the state
    // vector, x, at k = 430 (which is the last trustworthy state we can know)
    // State vector, x
    // x = [c; r(1:4); omega; theta(1:4)];

    Eigen::VectorXd x_polar_x(num_states); x_polar_x.setZero();
    Eigen::VectorXd x_polar_y(num_states); x_polar_y.setZero();
    Eigen::VectorXd x_polar_z(num_states); x_polar_z.setZero();
    Eigen::VectorXd x_polar_xaxis(num_states); x_polar_xaxis.setZero();
    Eigen::VectorXd x_polar_yaxis(num_states); x_polar_yaxis.setZero();
    Eigen::VectorXd x_polar_zaxis(num_states); x_polar_zaxis.setZero();
    Eigen::VectorXd x_polar_angle(num_states); x_polar_angle.setZero();
    x_polar_x(0) = x_init_x(0); // c(dc offset)
    x_polar_y(0) = x_init_y(0); // c(dc offset)
    x_polar_z(0) = x_init_z(0); // c(dc offset)
    x_polar_xaxis(0) = x_init_xaxis(0); // c(dc offset)
    x_polar_yaxis(0) = x_init_yaxis(0); // c(dc offset)
    x_polar_zaxis(0) = x_init_zaxis(0); // c(dc offset)
    x_polar_angle(0) = x_init_angle(0); // c(dc offset)

    for (size_t i = 1; i <= m; i++)
    {
        x_polar_x(i) = std::sqrt(std::pow(x_init_x(i), 2) + std::pow(x_init_x(i + m), 2)); // r_i, convert rectangular coords back to polar
        x_polar_y(i) = std::sqrt(std::pow(x_init_y(i), 2) + std::pow(x_init_y(i + m), 2)); // r_i, convert rectangular coords back to polar
        x_polar_z(i) = std::sqrt(std::pow(x_init_z(i), 2) + std::pow(x_init_z(i + m), 2)); // r_i, convert rectangular coords back to polar
        x_polar_xaxis(i) = std::sqrt(std::pow(x_init_xaxis(i), 2) + std::pow(x_init_xaxis(i + m), 2)); // r_i, convert rectangular coords back to polar
        x_polar_yaxis(i) = std::sqrt(std::pow(x_init_yaxis(i), 2) + std::pow(x_init_yaxis(i + m), 2)); // r_i, convert rectangular coords back to polar
        x_polar_zaxis(i) = std::sqrt(std::pow(x_init_zaxis(i), 2) + std::pow(x_init_zaxis(i + m), 2)); // r_i, convert rectangular coords back to polar
        x_polar_angle(i) = std::sqrt(std::pow(x_init_angle(i), 2) + std::pow(x_init_angle(i + m), 2)); // r_i, convert rectangular coords back to polar
    }

    x_polar_x(m + 1) = omega_0; // omega_0, (rad / sec)
    x_polar_y(m + 1) = omega_0; // omega_0, (rad / sec)
    x_polar_z(m + 1) = omega_0; // omega_0, (rad / sec)
    x_polar_xaxis(m + 1) = omega_0; // omega_0, (rad / sec)
    x_polar_yaxis(m + 1) = omega_0; // omega_0, (rad / sec)
    x_polar_zaxis(m + 1) = omega_0; // omega_0, (rad / sec)
    x_polar_angle(m + 1) = omega_0; // omega_0, (rad / sec)

    for (int i = 0; i < m; i++)
    {
        x_polar_x(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_x(i + m + 1), x_init_x(i + 1)); // theta = i*omega*T + phi, (rad)
        x_polar_y(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_y(i + m + 1), x_init_y(i + 1)); // theta = i*omega*T + phi, (rad)
        x_polar_z(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_z(i + m + 1), x_init_z(i + 1)); // theta = i*omega*T + phi, (rad)
        x_polar_xaxis(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_xaxis(i + m + 1), x_init_xaxis(i + 1)); // theta = i*omega*T + phi, (rad)
        x_polar_yaxis(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_yaxis(i + m + 1), x_init_yaxis(i + 1)); // theta = i*omega*T + phi, (rad)
        x_polar_zaxis(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_zaxis(i + m + 1), x_init_zaxis(i + 1)); // theta = i*omega*T + phi, (rad)
        x_polar_angle(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init_angle(i + m + 1), x_init_angle(i + 1)); // theta = i*omega*T + phi, (rad)
    }


    // return output
    std::shared_ptr< std::vector<double> > outputs(std::make_shared< std::vector<double> >());

    //outputs->insert(outputs->end(), z_init_x.data(), z_init_x.data() + z_init_x.size());
    //outputs->insert(outputs->end(), z_init_y.data(), z_init_y.data() + z_init_y.size());
    //outputs->insert(outputs->end(), z_init_z.data(), z_init_z.data() + z_init_z.size());
    //outputs->insert(outputs->end(), z_init_xaxis.data(), z_init_xaxis.data() + z_init_xaxis.size());
    //outputs->insert(outputs->end(), z_init_yaxis.data(), z_init_yaxis.data() + z_init_yaxis.size());
    //outputs->insert(outputs->end(), z_init_zaxis.data(), z_init_zaxis.data() + z_init_zaxis.size());
    //outputs->insert(outputs->end(), z_init_angle.data(), z_init_angle.data() + z_init_angle.size());

    outputs->insert(outputs->end(), x_init_x.data(), x_init_x.data() + x_init_x.size());
    outputs->insert(outputs->end(), x_init_y.data(), x_init_y.data() + x_init_y.size());
    outputs->insert(outputs->end(), x_init_z.data(), x_init_z.data() + x_init_z.size());
    outputs->insert(outputs->end(), x_init_xaxis.data(), x_init_xaxis.data() + x_init_xaxis.size());
    outputs->insert(outputs->end(), x_init_yaxis.data(), x_init_yaxis.data() + x_init_yaxis.size());
    outputs->insert(outputs->end(), x_init_zaxis.data(), x_init_zaxis.data() + x_init_zaxis.size());
    outputs->insert(outputs->end(), x_init_angle.data(), x_init_angle.data() + x_init_angle.size());
    outputs->insert(outputs->end(), x_polar_x.data(), x_polar_x.data() + x_polar_x.size());
    outputs->insert(outputs->end(), x_polar_y.data(), x_polar_y.data() + x_polar_y.size());
    outputs->insert(outputs->end(), x_polar_z.data(), x_polar_z.data() + x_polar_z.size());
    outputs->insert(outputs->end(), x_polar_xaxis.data(), x_polar_xaxis.data() + x_polar_xaxis.size());
    outputs->insert(outputs->end(), x_polar_yaxis.data(), x_polar_yaxis.data() + x_polar_yaxis.size());
    outputs->insert(outputs->end(), x_polar_zaxis.data(), x_polar_zaxis.data() + x_polar_zaxis.size());
    outputs->insert(outputs->end(), x_polar_angle.data(), x_polar_angle.data() + x_polar_angle.size());

//    qint64 elNsec = elTimer.nsecsElapsed();

//    qDebug() << "Nsec elapsed:" << elNsec;

    return outputs;
}

void CyclicModel::loadData(QString filename, std::vector<double> &X)
{
    // check if the directory exists
    QFile inFile(filename);

    if( inFile.open(QIODevice::ReadOnly) )
        qDebug() << "Opened inFile" << filename;

    QTextStream in(&inFile);

    double dummy;
    while(!in.atEnd())
    {
        in >> dummy;
        X.push_back(dummy);
    }
    qDebug() << "Read" << X.size() << "points.";
}

void CyclicModel::load4x4Data(QString filename, EigenDequeVector7d &X)
{
    // check if the directory exists
    QFile inFile(filename);

    if( inFile.open(QIODevice::ReadOnly) )
        qDebug() << "Opened inFile" << filename;

    QTextStream in(&inFile);

    double dummy;
    EigenAffineTransform3d tempT;

    while(!in.atEnd())
    {
        for(size_t i = 0; i < 4; i++)
        {
            for(size_t j = 0; j < 4; j++)
            {
                if(in.atEnd())
                {
                    qDebug() << "Early termination!" << filename;
                    return;
                }

                in >> dummy;

                if(i < 3)
                    tempT(i,j) = dummy;
            }
        }

        // axis angle
        EigenVector7d temp7d;
        Eigen::AngleAxisd tempEA(tempT.rotation());

        temp7d << tempT(0,3), tempT(1,3), tempT(2,3),
                  tempEA.axis()(0), tempEA.axis()(1), tempEA.axis()(2), tempEA.angle();

        // push newest readings
        X.push_back(temp7d);

    }
    qDebug() << "Read" << X.size() << "points.";
}

void CyclicModel::saveFilteredData(QString filename, const std::vector<double> &Y)
{
    QFile outFile(filename);

    if( outFile.open(QIODevice::WriteOnly) )
        qDebug() << "Opened outFile" << filename;

    QTextStream out(&outFile);

    for(size_t i = 0; i < Y.size(); i++)
        out << QString::number(Y[i], 'f', 4) << "\n";

    qDebug() << "Wrote" << Y.size() << "points.";
}

void CyclicModel::save4x4FilteredData(QString filename, const EigenDequeVector7d &Y)
{
    QFile outFile(filename);

    if( outFile.open(QIODevice::WriteOnly) )
        qDebug() << "Opened outFile" << filename;

    QTextStream out(&outFile);

    for(size_t i = 0; i < Y.size(); i++)
    {
        out << QString::number(Y[i](0), 'f', 4) << " "
            << QString::number(Y[i](1), 'f', 4) << " "
            << QString::number(Y[i](2), 'f', 4) << " "
            << QString::number(Y[i](3), 'f', 4) << " "
            << QString::number(Y[i](4), 'f', 4) << " "
            << QString::number(Y[i](5), 'f', 4) << " "
            << QString::number(Y[i](6), 'f', 4) << "\n";
    }

    qDebug() << "Wrote" << Y.size() << "points.";
}

void CyclicModel::testLPF()
{
    //FIR lpf (Low_pass (60, 150, FILTER_ORDER));
    QString inputFile("D:\\Dropbox\\Harvard\\ICEbot share\\Current working directory\\2016-12-11 Testing Cpp FiltFilt\\data_to_filter_x.txt");
    QString outputFile("D:\\Dropbox\\Harvard\\ICEbot share\\Current working directory\\2016-12-11 Testing Cpp FiltFilt\\filtered_data_x_Cpp.txt");

    filtfilt lpf;
    std::vector<double> X,Y;

    loadData(inputFile, X);
    X.pop_back(); // delete the last 0

//    for(auto x : X)
//        std::cout << x << std::endl;

    QElapsedTimer elTimer;
    elTimer.start();

    lpf.run(X, Y);
    lpf.run(X, Y);
    lpf.run(X, Y);
    lpf.run(X, Y);
    lpf.run(X, Y);

    qint64 elNsec = elTimer.nsecsElapsed();
    qDebug() << "\nNsec elapsed:" << elNsec;

    saveFilteredData(outputFile, Y);

    // Test 7 axis filtering
    QString inputFile7d("D:\\Dropbox\\Harvard\\ICEbot share\\Current working directory\\2016-12-11 Testing Cpp FiltFilt\\data_to_filter.txt");
    QString outputFile7d("D:\\Dropbox\\Harvard\\ICEbot share\\Current working directory\\2016-12-11 Testing Cpp FiltFilt\\filtered_data_Cpp.txt");

    EigenDequeVector7d X_7d, Y_7d;

    load4x4Data(inputFile7d, X_7d);

    elTimer.restart();

    lpf.run(X_7d, Y_7d);

    elNsec = elTimer.nsecsElapsed();
    qDebug() << "\n>>> 7d Nsec elapsed:" << elNsec;

    save4x4FilteredData(outputFile7d, Y_7d);

}
