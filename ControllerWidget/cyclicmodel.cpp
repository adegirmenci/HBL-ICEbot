#include "cyclicmodel.h"

CyclicModel::CyclicModel()
{
    resetModel();

    std::cout << "--- Initialized CyclicModel. ---" << std::endl;
}

CyclicModel::~CyclicModel()
{

}

void CyclicModel::resetModel()
{
    // ------------------ EMPTY BUFFERS ------------------ //

    // UNFILTERED DATA
    m_BBfixed_CT   .clear();
    m_BBfixed_Instr.clear();
    m_BBfixed_BB   .clear();

    m_Bird4    .clear();
    m_Bird4_new.clear();

    // TIME DATA
    m_timeData_init.clear();
    m_timeData_new .clear();

    // ------------------ INIT BUFFERS ------------------ //

    // UNFILTERED DATA
    m_BBfixed_CT   .reserve(N_SAMPLES); // this stores all of the incoming CT points
    m_BBfixed_Instr.reserve(N_SAMPLES); // this stores all of the incoming instr_x points
    m_BBfixed_BB   .reserve(N_SAMPLES); // this stores all of the incoming BB points
//    m_BBfixed_CT   .resize(7, std::vector<double>()); // this stores all of the incoming CT points
//    m_BBfixed_Instr.resize(7, std::vector<double>()); // this stores all of the incoming instr_x points
//    m_BBfixed_BB   .resize(7, std::vector<double>()); // this stores all of the incoming BB points

    m_Bird4    .reserve(N_SAMPLES);     // this remains constant after initialization
    m_Bird4_new.resize(N_SAMPLES, 0.0); // most recent chest tracker data

    // FILTERED DATA
    m_BBfixed_CT_filtered = EigenMatrixFiltered::Zero(N_FILTERED,7); // this remains constant after initialization
    m_BBfixed_BB_filtered = EigenMatrixFiltered::Zero(N_FILTERED,7); // this remains constant after initialization
    m_Bird4_filtered      = EigenVectorFiltered::Zero(N_FILTERED); // this remains constant after initialization
    m_Bird4_filtered_new  = EigenVectorFiltered::Zero(N_FILTERED); // most recent filtered chest tracker data

    // ***CURRENT*** RECTANGULAR COMPONENTS
    m_BBfixed_CT_rectangular = EigenMatrixRectangular::Zero(); // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    m_BBfixed_BB_rectangular = EigenMatrixRectangular::Zero(); // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    m_Bird4_rectangular      = EigenVectorRectangular::Zero(); // 1 component : x (benchtop) or -z (in vivo)

    // ***CURRENT*** POLAR COMPONENTS
    m_BBfixed_CT_polar = EigenMatrixPolar::Zero(); // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    m_BBfixed_BB_polar = EigenMatrixPolar::Zero(); // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    m_Bird4_polar      = EigenVectorPolar::Zero(); // 1 component : x (benchtop) or -z (in vivo)

    // TIME DATA
    m_timeData_init.reserve(N_SAMPLES);    // stores the time vector for the model initialization observations
    m_timeData_new .resize(N_SAMPLES, 0.0); // stores time for the most recent observations

    m_omega0 = 0.0; // frequency

    m_numSamples = 0; // number of observations added IN TOTAL

    m_isTrained = false; // is the model trained
    m_lastTrainingTimestamp = 0.0; // when last training was performed
    m_isInVivo = false; // are we in IN VIVO mode
}

void CyclicModel::addTrainingObservation(const EigenAffineTransform3d &T_BB_CT_curTipPos,
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

    // local_BB_Box * local_Box_BBmobile = em_base; // T_BB_Box * T_Box_BBmobile = T_BBfixed_BBmobile
    // BBfixed_CT.insert(BBfixed_CT.end(), em_tip, em_tip+16);
    // BBfixed_Instr.insert(BBfixed_Instr.end(), em_instr, em_instr+16);
    // BBfixed_BB.insert(BBfixed_BB.end(), em_base, em_base+16);
    // Bird4.insert(Bird4.end(), local_Bird4, local_Bird4+16);

    if( m_numSamples < N_SAMPLES)
    {
        EigenAffineTransform3d T_BB_BBm = T_BB_Box * T_Box_BBmobile;

        // axis angle
        Eigen::AngleAxisd tempEA_CT  (T_BB_CT_curTipPos.rotation()),
                          tempEA_Inst(T_BB_targetPos.rotation()),
                          tempEA_BB  (T_BB_BBm.rotation()); //, tempEA_Bird(T_Bird4.rotation());

        // Create 7d Eigen::Vectors to hold the x,y,z,xaxis,yaxis,zaxis,angle
        EigenVector7d tempCT, tempInst, tempBB;

        tempCT << T_BB_CT_curTipPos(0,3), T_BB_CT_curTipPos(1,3), T_BB_CT_curTipPos(2,3),
                  tempEA_CT.axis()(0), tempEA_CT.axis()(1), tempEA_CT.axis()(2), tempEA_CT.angle();

        tempInst << T_BB_targetPos(0,3), T_BB_targetPos(1,3), T_BB_targetPos(2,3),
                  tempEA_Inst.axis()(0), tempEA_Inst.axis()(1), tempEA_Inst.axis()(2), tempEA_Inst.angle();

        tempBB << T_BB_BBm(0,3), T_BB_BBm(1,3), T_BB_BBm(2,3),
                  tempEA_BB.axis()(0), tempEA_BB.axis()(1), tempEA_BB.axis()(2), tempEA_BB.angle();

        double tempBird4;
        if(m_isInVivo)
            tempBird4 = -T_Bird4(2,3); // -z axis of the EM tracker (in vivo)
        else
            tempBird4 = T_Bird4(0,3); //  x axis of the EM tracker (benchtop)

        // push newest readings
        m_BBfixed_CT.push_back(tempCT);
        m_BBfixed_Instr.push_back(tempInst);
        m_BBfixed_BB.push_back(tempBB);
        m_Bird4.push_back(tempBird4);
        m_timeData_init.push_back(sampleTime);
//        for(size_t i = 0; i<7; i++)
//        {
//            m_BBfixed_CT[i].push_back(tempCT(i));
//            m_BBfixed_Instr[i].push_back(tempInst(i));
//            m_BBfixed_BB[i].push_back(tempBB(i));
//        }
//        m_Bird4.push_back(tempBird4);
//        m_timeData_init.push_back(sampleTime);

        m_numSamples++;
    }
    else
    {
        printf("Already have enough training points!\n");
        printf("I'll forgive you and fix this.\n");

        printf("Train model and call addObservation.\n");
        trainModel();
        addObservation(T_Bird4, sampleTime);
    }
}

void CyclicModel::addObservation(const EigenAffineTransform3d &T_Bird4, const double sampleTime)
{
    if(m_isTrained)
    {

        // erase oldest observation
        m_Bird4_new   .erase(m_Bird4_new.begin());
        m_timeData_new.erase(m_timeData_new.begin());

        double tempBird4;
        if(m_isInVivo)
            tempBird4 = -T_Bird4(2,3); // -z axis of the EM tracker (in vivo)
        else
            tempBird4 = T_Bird4(0,3); //  x axis of the EM tracker (benchtop)

        // add new observation
        m_Bird4_new.push_back(tempBird4);
        m_timeData_new.push_back(sampleTime);

        m_numSamples++;

        updatePeriod( peakDetector(false) );
    }
    else
    {
        printf("Not trained yet!\n");
        printf("FATAL ERROR!!!\n"); // becuase we lost this data
    }
}

void CyclicModel::trainModel()
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
        size_t num_states = N_STATES;
        size_t N_filtered = N_initpts - 2*edge_effect;

        // low pass filter the data
        filterTrainingData();

        // Use peak detection to look at the filtered data and find the period
        double period = peakDetector(true); // run for init

        // omega_0 = 2*pi*1/period; % 2*pi*breathing frequency (rad/sec)
        m_omega0 = 2.0*pi/period;

        // Step 1. z = Ax + noise.  Assemble A and z.  Use rectangular fourier series.
        // use <<< cycle_recalculate >>> here

        // ************************************************************* //
        //                                                               //
        // copy all of the init data over to the containers for new data //
        //                                                               //
        // ************************************************************* //

        m_isTrained = true;
    }
}

void CyclicModel::updatePeriod(const double shift)
{

}

double CyclicModel::getPrediction(const double timeShift, const EigenVectorPolar &x_polar, const EigenVectorRectangular &x_rect)
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

void CyclicModel::setInVivo(const bool isInVivo)
{
    if( (!m_isTrained) && (m_numSamples == 0) )
        m_isInVivo = isInVivo;
    else
        printf("Can't modify mode now!");
}

void CyclicModel::retrainModel()
{

}

void CyclicModel::filterTrainingData()
{
    m_LowPassFilter.run(m_BBfixed_CT, m_BBfixed_CT_filtered);
    //m_LowPassFilter.run(m_BBfixed_Instr, m_BBfixed_Instr_filtered);
    m_LowPassFilter.run(m_BBfixed_BB, m_BBfixed_BB_filtered);
    m_LowPassFilter.run(m_Bird4, m_Bird4_filtered);

//    //m_BBfixed_CT_filtered = EigenMatrixFiltered::Zero(N_FILTERED,7);

//    EigenVectorFiltered temp;
//    for(size_t i = 0; i<7; i++)
//    {
//        m_LowPassFilter.run(m_BBfixed_CT[i], temp);
//        m_BBfixed_CT_filtered.col(i) = temp;
//        m_LowPassFilter.run(m_BBfixed_BB[i], temp);
//        m_BBfixed_BB_filtered.col(i) = temp;
//    }
//    m_LowPassFilter.run(m_Bird4, m_Bird4_filtered);
}

void CyclicModel::filterNewObservations()
{

}

double CyclicModel::peakDetector(const bool runForInit)
{
    // look at Bird4
    // EigenVectorFiltered::Index maxRow, maxCol, minRow, minCol;
    auto maxBird = m_Bird4_filtered.maxCoeff();//(&maxRow, &maxCol);
    auto minBird = m_Bird4_filtered.minCoeff();
    double diffBird = maxBird - minBird;
    double thresh = minBird + diffBird * PEAK_THRESHOLD;

    // find indices that are greater than thresh
    std::vector<size_t> peakIdx; // container for indices
    auto it = std::find_if(m_Bird4_filtered.data(), m_Bird4_filtered.data() + m_Bird4_filtered.size(),
                           [thresh](double i){return i > thresh;});
    while (it != (m_Bird4_filtered.data() + m_Bird4_filtered.size()) ) {
       peakIdx.emplace_back(std::distance(m_Bird4_filtered.data(), it));
       it = std::find_if(std::next(it), m_Bird4_filtered.data() + m_Bird4_filtered.size(),
                         [thresh](double i){return i > thresh;});
    }

    // get the timestamp at peaks
    std::vector<double> peaksTime; peaksTime.reserve(peakIdx.size());
    if(runForInit){
        for(auto it = peakIdx.begin(); it != peakIdx.end(); ++it){
            peaksTime.emplace_back(m_timeData_init[*it]);
        }
    }
    else{
        for(auto it = peakIdx.begin(); it != peakIdx.end(); ++it){
            peaksTime.emplace_back(m_timeData_new[*it]);
        }
    }

    // take the difference of time stamps
    std::vector<double> peaksTdiff; peaksTdiff.reserve(peaksTime.size() - 1);
    std::transform(peaksTime.begin()+1,peaksTime.end(),
                   peaksTime.begin(),std::back_inserter(peaksTdiff),std::minus<double>());

    // find time differences larger than 1 seconds
    double largeTimeGap = 1000.0; // milliseconds
    std::vector<size_t> peakGapsIdx; // container for indices
    auto it2 = std::find_if(std::begin(peaksTdiff), std::end(peaksTdiff),
                            [largeTimeGap](double i){return i > largeTimeGap;});
    while (it2 != std::end(peaksTdiff)) {
       peakGapsIdx.emplace_back(std::distance(std::begin(peaksTdiff), it2));
       it2 = std::find_if(std::next(it2), std::end(peaksTdiff),
                          [largeTimeGap](double i){return i > largeTimeGap;});
    }

    // we better have peaks
    if(peakGapsIdx.size() < 1)
        printf("PeakDetector is in trouble!\n");

    std::vector<double> peakTimesRight, peakTimesLeft;
    peakTimesLeft.emplace_back(peaksTime.front());
    for(auto it3 = peakGapsIdx.begin(); it3 != peakGapsIdx.end(); ++it3){
        peakTimesRight.emplace_back(peaksTime[*it3]);
        peakTimesLeft.emplace_back(peaksTime[1 + *it3]);
    }
    peakTimesRight.emplace_back(peaksTime.back());

//    if peak_detect_me(1) > tops_thresh
//        tops_peak_times_right(1)=[];
//        tops_peak_times_left(1)=[];
//    end
//    if peak_detect_me(end) > tops_thresh
//        tops_peak_times_right(end)=[];
//        tops_peak_times_left(end)=[];
//    end
    if( m_Bird4_filtered.head(1)[0] > thresh )
    {
        peakTimesRight.erase(peakTimesRight.begin());
        peakTimesLeft.erase(peakTimesLeft.begin());
    }
    if( m_Bird4_filtered.tail(1)[0] > thresh )
    {
        peakTimesRight.pop_back();
        peakTimesLeft.pop_back();
    }

    // better not be empty
    if(peakTimesLeft.size() < 1)
        std::printf("PeakDetector is in trouble 2!\n");

    // tops_peak_times_means = mean([tops_peak_times_right tops_peak_times_left],2);
    std::vector<double> mean; mean.reserve(peakTimesRight.size());
    std::transform(peakTimesRight.begin(), peakTimesRight.end(),
                   peakTimesLeft.begin(), std::back_inserter(mean),
                   [](double l, double r){ return (l+r)/2.0; });

//    if length(tops_peak_times_means)>1
//        tops_peak_times_diffs = tops_peak_times_means(2:end)-tops_peak_times_means(1:(end-1));
//        period=mean(tops_peak_times_diffs);
//    else
//        period=breath_expected;
//    end
    double period = 0.0;

    if(mean.size() > 1)
    {
        period = std::accumulate(mean.begin(),mean.end(),0.0) / (double)mean.size();
    }
    else
    {
        period = mean[0];
    }

    // In vivo version has an error checker to make sure the breathing period is
    // between 4 - 5.5 seconds
    if(m_isInVivo)
    {
        if( period < (BREATH_RATE*0.9) )
            period = BREATH_RATE;
        else if( period > (BREATH_RATE*1.1) )
            period = BREATH_RATE;
    }

    return period;
}

void CyclicModel::cycle_recalculate(const std::vector<double> &inputs)
{
//    QElapsedTimer elTimer;
//    elTimer.start();

    // TODO: add error checking to ensure that the vector size is correct

    std::vector<double>::const_iterator iter = inputs.begin();

    // get all of the constants
    size_t m = N_HARMONICS; // m = number of sinusoid components
    double delta_t = SAMPLE_DELTA_TIME; // Time step for each collected data point
    size_t N_initpts = N_SAMPLES;  // number of initialization points
    size_t edge_effect = EDGE_EFFECT;
    double omega_0 = m_omega0;

    // calculate things from inputs
    size_t num_states = m * 2 + 2;
    size_t N_filtered = N_initpts - 2 * edge_effect;

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

void CyclicModel::load4x4Data(QString filename, EigenStdVecVector7d &X)
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

void CyclicModel::load4x4Data(QString filename, std::vector<std::vector<double> > &X)
{
    // check if the directory exists
    QFile inFile(filename);

    if( inFile.open(QIODevice::ReadOnly) )
        qDebug() << "Opened inFile" << filename;

    QTextStream in(&inFile);

    double dummy;
    EigenAffineTransform3d tempT;

    X.clear();
    X.resize(7,std::vector<double>());

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

        // push newest readings
        X[0].push_back(tempT(0,3));
        X[1].push_back(tempT(1,3));
        X[2].push_back(tempT(2,3));
        X[3].push_back(tempEA.axis()(0));
        X[4].push_back(tempEA.axis()(1));
        X[5].push_back(tempEA.axis()(2));
        X[6].push_back(tempEA.angle());
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

void CyclicModel::saveFilteredData(QString filename, const EigenVectorFiltered &Y)
{
    QFile outFile(filename);

    if( outFile.open(QIODevice::WriteOnly) )
        qDebug() << "Opened outFile" << filename;

    QTextStream out(&outFile);

    for(size_t i = 0; i < Y.size(); i++)
        out << QString::number(Y[i], 'f', 4) << "\n";

    qDebug() << "Wrote" << Y.size() << "points.";
}

void CyclicModel::save4x4FilteredData(QString filename, const EigenStdVecVector7d &Y)
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

void CyclicModel::save4x4FilteredData(QString filename, const EigenMatrixFiltered &Y)
{
    QFile outFile(filename);

    if( outFile.open(QIODevice::WriteOnly) )
        qDebug() << "Opened outFile" << filename;

    QTextStream out(&outFile);

    for(size_t i = 0; i < Y.rows(); i++)
    {
        out << QString::number(Y(i,0), 'f', 4) << " "
            << QString::number(Y(i,1), 'f', 4) << " "
            << QString::number(Y(i,2), 'f', 4) << " "
            << QString::number(Y(i,3), 'f', 4) << " "
            << QString::number(Y(i,4), 'f', 4) << " "
            << QString::number(Y(i,5), 'f', 4) << " "
            << QString::number(Y(i,6), 'f', 4) << "\n";
    }

    qDebug() << "Wrote" << Y.rows() << "points.";
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
    qDebug() << "\nNsec elapsed:" << elNsec/5;

    saveFilteredData(outputFile, Y);

    // test  run(const std::vector<double> &X, EigenVectorFiltered &Y);
    EigenVectorFiltered Y2;

    elTimer.restart();

    lpf.run(X, Y2);
    lpf.run(X, Y2);
    lpf.run(X, Y2);
    lpf.run(X, Y2);
    lpf.run(X, Y2);

    elNsec = elTimer.nsecsElapsed();
    qDebug() << "\n>>> EigenVectorFiltered Nsec elapsed:" << elNsec/5;

    //saveFilteredData(outputFile, Y2);

    // Test 7 axis filtering
    QString inputFile7d("D:\\Dropbox\\Harvard\\ICEbot share\\Current working directory\\2016-12-11 Testing Cpp FiltFilt\\data_to_filter.txt");
    QString outputFile7d("D:\\Dropbox\\Harvard\\ICEbot share\\Current working directory\\2016-12-11 Testing Cpp FiltFilt\\filtered_data_Cpp.txt");

    EigenStdVecVector7d X_7dv, Y_7dv;
    load4x4Data(inputFile7d, X_7dv);

    elTimer.restart();

    lpf.run(X_7dv, Y_7dv);

    elNsec = elTimer.nsecsElapsed();
    qDebug() << "\n>>> 7d Nsec elapsed:" << elNsec;

    //save4x4FilteredData(outputFile7d, Y_7dv);

    // test  run(const std::vector<double> &X, EigenVectorFiltered &Y);
    EigenMatrixFiltered Y3;

    elTimer.restart();

    lpf.run(X_7dv, Y3);

    elNsec = elTimer.nsecsElapsed();
    qDebug() << "\n>>> EigenMatrixFiltered Nsec elapsed:" << elNsec;

    save4x4FilteredData(outputFile7d, Y3);

    //
    std::vector<std::vector<double>> Xvv;
    load4x4Data(inputFile7d, Xvv);

    elTimer.restart();

    EigenVectorFiltered temp;
    for(size_t i = 0; i<7; i++)
    {
        lpf.run(Xvv[i], temp);
        Y3.col(i) = temp;
    }

    elNsec = elTimer.nsecsElapsed();
    qDebug() << "\n>>> std::vector<std::vector<double>> Nsec elapsed:" << elNsec;

    //save4x4FilteredData(outputFile7d, Y3);
}
