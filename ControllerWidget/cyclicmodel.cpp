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

    m_Bird4    .reserve(N_SAMPLES);     // this remains constant after initialization
    m_Bird4_new.resize(N_SAMPLES, 0.0); // most recent chest tracker data

    // FILTERED DATA
    m_BBfixed_CT_filtered = EigenMatrixFiltered::Zero(N_FILTERED,7); // this remains constant after initialization
    m_BBfixed_BB_filtered = EigenMatrixFiltered::Zero(N_FILTERED,7); // this remains constant after initialization
    m_Bird4_filtered      = EigenVectorFiltered::Zero(N_FILTERED); // this remains constant after initialization
    m_Bird4_filtered_new  = EigenVectorFiltered::Zero(N_FILTERED); // most recent filtered chest tracker data
    m_breathSignalFromModel = EigenVectorFiltered::Zero(N_FILTERED);

    // ***CURRENT*** RECTANGULAR COMPONENTS
    m_BBfixed_CT_rectangular = EigenMatrixRectangular::Zero(N_RECT,7); // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    m_BBfixed_BB_rectangular = EigenMatrixRectangular::Zero(N_RECT,7); // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    m_Bird4_rectangular      = EigenVectorRectangular::Zero(N_RECT); // 1 component : x (benchtop) or -z (in vivo)

    // ***CURRENT*** POLAR COMPONENTS
    m_BBfixed_CT_polar = EigenMatrixPolar::Zero(N_POLAR,7); // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    m_BBfixed_BB_polar = EigenMatrixPolar::Zero(N_POLAR,7); // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    m_Bird4_polar      = EigenVectorPolar::Zero(N_POLAR); // 1 component : x (benchtop) or -z (in vivo)

    // TIME DATA
    m_timeData_init.reserve(N_SAMPLES);    // stores the time vector for the model initialization observations
    m_timeData_new .resize(N_SAMPLES, 0.0); // stores time for the most recent observations

    m_nFutureSamples = EDGE_EFFECT;

    m_omega0 = 2.*pi/BREATH_RATE; // frequency

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
        QElapsedTimer elTimer;
        elTimer.start();

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

        qint64 elNsec = elTimer.nsecsElapsed();
        printf("Insert new obs: %d ns\n", elNsec);

        // up to here take about 3 microseconds

        // Retrain model
        std::cout << "Retrain model ";
        retrainModel();
        std::cout << "done." << std::endl;
    }
    else
    {
        std::cout << "Not trained yet!\n" << std::endl;
        std::cout << "FATAL ERROR!!!\n" << std::endl; // becuase we lost this data
    }
}

void CyclicModel::trainModel()
{
    if(m_numSamples < N_SAMPLES) // not enough samples
    {
        std::cout << "m_numSamples is less than N_SAMPLES!" << std::endl;

        m_isTrained = false;
    }
    else if(m_isTrained) // already trained
    {
        std::cout << "Already trained!" << std::endl;
    }
    else // train
    {
        // size_t m = N_HARMONICS; // m = number of sinusoid components
        // double delta_t = SAMPLE_DELTA_TIME; // Time step for each collected data point
        size_t N_initpts = m_BBfixed_CT.size(); // number of initialization points

        std::cout << "Training model.\n" << std::endl;

        if(N_initpts <= 2*EDGE_EFFECT)
        {
            std::cout << "Error: N_initpts < 2*edge_effect.\n" << std::endl;
            return;
        }
        if(N_initpts != N_SAMPLES)
        {
            std::cout << "Error: N_initpts != N_SAMPLES\n" << std::endl;
        }

        // calculate things from inputs
        // size_t num_states = N_STATES;
        // size_t N_filtered = N_initpts - 2*EDGE_EFFECT;

        // low pass filter the data
        filterTrainingData();
        std::cout << "Filtered training data.\n" << std::endl;

//        // save filtered data
//        QString outputFile1("C:\\Users\\Alperen\\Documents\\QT Projects\\ICEbot_QT_v1\\LoggedData\\m_BBfixed_CT_filtered.txt");
//        QString outputFile2("C:\\Users\\Alperen\\Documents\\QT Projects\\ICEbot_QT_v1\\LoggedData\\m_BBfixed_BB_filtered.txt");
//        QString outputFile3("C:\\Users\\Alperen\\Documents\\QT Projects\\ICEbot_QT_v1\\LoggedData\\m_Bird4_filtered.txt");
//        save4x4FilteredData(outputFile1, m_BBfixed_CT_filtered);
//        save4x4FilteredData(outputFile2, m_BBfixed_BB_filtered);
//        saveFilteredData(outputFile3, m_Bird4_filtered);

        // Use peak detection to look at the filtered data and find the period
        updatePeriod( peakDetector(true) ); // run for init
        std::cout << "Updated period.\n" << std::endl;

        // Get Fourier decomposition

        // TODO : should compare if the column by column computation is slower than the whole matrix approach
//        Eigen::VectorXd z_init_x = m_BBfixed_CT_filtered.col(0);
//        Eigen::VectorXd z_init_y = m_BBfixed_CT_filtered.col(1);
//        Eigen::VectorXd z_init_z = m_BBfixed_CT_filtered.col(2);
//        Eigen::VectorXd z_init_xaxis = m_BBfixed_CT_filtered.col(3);
//        Eigen::VectorXd z_init_yaxis = m_BBfixed_CT_filtered.col(4);
//        Eigen::VectorXd z_init_zaxis = m_BBfixed_CT_filtered.col(5);
//        Eigen::VectorXd z_init_angle = m_BBfixed_CT_filtered.col(6);

//        cycle_recalculate(m_BBfixed_CT_filtered, m_BBfixed_CT_rectangular, m_BBfixed_CT_polar);
//        cycle_recalculate(m_BBfixed_BB_filtered, m_BBfixed_BB_rectangular, m_BBfixed_BB_polar);
//        cycle_recalculate(m_Bird4_filtered, m_Bird4_rectangular, m_Bird4_polar);

        // TODO : is this faster?
//        m_BBfixed_CT_polarRect = cycle_recalculate_concurrentM(m_BBfixed_CT_filtered, m_omega0);
//        m_BBfixed_BB_polarRect = cycle_recalculate_concurrentM(m_BBfixed_BB_filtered, m_omega0);
//        m_Bird4_polarRect = cycle_recalculate_concurrentV(m_Bird4_filtered, m_omega0);

        mConcurrent1 = QtConcurrent::run(this, &CyclicModel::cycle_recalculate_concurrentM, m_BBfixed_CT_filtered, m_omega0);
        mConcurrent2 = QtConcurrent::run(this, &CyclicModel::cycle_recalculate_concurrentM, m_BBfixed_BB_filtered, m_omega0);
        mConcurrent3 = QtConcurrent::run(this, &CyclicModel::cycle_recalculate_concurrentV, m_Bird4_filtered, m_omega0);

        // 3 should finish first, process that while the others are running
        mConcurrent3.waitForFinished();
        m_Bird4_polarRect = mConcurrent3.result();

        m_Bird4_polar       = m_Bird4_polarRect.segment(0, N_POLAR);
        m_Bird4_rectangular = m_Bird4_polarRect.segment(N_POLAR, N_RECT);

        std::cout << "m_Bird4_polarRect\n" << m_Bird4_polarRect << std::endl;
        std::cout << "m_Bird4_polar\n" << m_Bird4_polar << std::endl;
        std::cout << "m_Bird4_rectangular\n" << m_Bird4_rectangular << std::endl;

        // 1 should be done sooner since it was launched first
        mConcurrent1.waitForFinished();
        m_BBfixed_CT_polarRect = mConcurrent1.result();

        m_BBfixed_CT_polar       = m_BBfixed_CT_polarRect.block(0,0, N_POLAR, 7);
        m_BBfixed_CT_rectangular = m_BBfixed_CT_polarRect.block(N_POLAR,0, N_RECT, 7);

        // 2 should be done
        mConcurrent2.waitForFinished();
        m_BBfixed_BB_polarRect = mConcurrent2.result();

        m_BBfixed_BB_polar       = m_BBfixed_BB_polarRect.block(0,0, N_POLAR, 7);
        m_BBfixed_BB_rectangular = m_BBfixed_BB_polarRect.block(N_POLAR,0, N_RECT, 7);

        std::cout << "Model created.\n" << std::endl;

        // ************************************************************* //
        //                                                               //
        // copy all of the init data over to the containers for new data //
        //                                                               //
        // ************************************************************* //

        m_Bird4_new = m_Bird4; // deep copy
        m_timeData_new = m_timeData_init; // deep copy

        // training is done!
        m_isTrained = true;
    }
}


void CyclicModel::retrainModel()
{
    if(!m_isTrained)
    {
        std::cerr << "Not trained yet!" << std::endl;
    }
    else
    {
        QElapsedTimer elTimer;
        elTimer.start();

        // low pass filter the data
        filterNewObservations();

        qint64 elNsec = elTimer.nsecsElapsed();
        printf("filterNewObservations: %d ns\n", elNsec);
        elTimer.restart();

        // calculate expected values from model
//        // first copy the time list into an array
//        copy(time_data_current.begin(),time_data_current.end(),&time_data_current_array[0]);
//        // then put the time array into the loop
//        for (int i=0; i<cycle_data_size; i++){
//            t_minus_t_begin=time_data_current_array[i]-t_begin;

//            if (m_flag_invivo_mode == 0){
//                breathing_signal_value = cycle_model_predict(t_minus_t_begin, local_model_rect, local_model_polar);
//            }
//            else{
//                breathing_signal_value = -1*cycle_model_predict(t_minus_t_begin, local_model_rect, local_model_polar);
//            }

//            breathing_signal_model.push_back(breathing_signal_value);
//        }

        // update the prediction of m_breathSignalFromModel
        double t_minus_t_begin;
        for(size_t i = 0; i < N_FILTERED; i++)
        {
            t_minus_t_begin = m_timeData_new[i+EDGE_EFFECT] - m_timeData_init[0];

            if(m_isInVivo)
            {
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                // FIXME : do we need to differentiate b/w in vivo and benchtop?
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                m_breathSignalFromModel(i) = -1.0*getPrediction(t_minus_t_begin, m_Bird4_polar, m_Bird4_rectangular);
                // WE'RE NOT SURE IF THIS -1.0 IS SUPPOSED TO BE HERE !!!!!!!!!!
            }
            else
            {

                m_breathSignalFromModel(i) = getPrediction(t_minus_t_begin, m_Bird4_polar, m_Bird4_rectangular);
            }
        }

        // update period / freq
        updatePeriod( peakDetector(false) );

        elNsec = elTimer.nsecsElapsed();
        printf("Update period: %d ns\n", elNsec);
        elTimer.restart();

        // update Fourier components
//        cycle_recalculate(m_BBfixed_CT_filtered, m_BBfixed_CT_rectangular, m_BBfixed_CT_polar);
//        cycle_recalculate(m_BBfixed_BB_filtered, m_BBfixed_BB_rectangular, m_BBfixed_BB_polar);
//        cycle_recalculate(m_Bird4_filtered, m_Bird4_rectangular, m_Bird4_polar);

        // TODO : run these concurrently
//        Eigen::MatrixXd m_BBfixed_CT_polarRect = cycle_recalculate_concurrent(m_BBfixed_CT_filtered, m_omega0);
//        Eigen::MatrixXd m_BBfixed_BB_polarRect = cycle_recalculate_concurrent(m_BBfixed_BB_filtered, m_omega0);
//        Eigen::VectorXd m_Bird4_polarRect = cycle_recalculate_concurrent(m_Bird4_filtered, m_omega0);


//        std::cout << "mConcurrent1.resultCount() " << mConcurrent1.resultCount() << mConcurrent1.isRunning() << mConcurrent1.isFinished() << std::endl;
//        std::cout << "mConcurrent2.resultCount() " << mConcurrent2.resultCount() << mConcurrent2.isRunning() << mConcurrent2.isFinished()<< std::endl;
//        std::cout << "mConcurrent3.resultCount() " << mConcurrent3.resultCount() << mConcurrent3.isRunning() << mConcurrent3.isFinished()<< std::endl;


        if( (mConcurrent1.resultCount() > 0) && (mConcurrent2.resultCount() > 0) && (mConcurrent3.resultCount() > 0) )
        {
            m_BBfixed_CT_polarRect = mConcurrent1.result();
            mConcurrent1 = QtConcurrent::run(this, &CyclicModel::cycle_recalculate_concurrentM, m_BBfixed_CT_filtered, m_omega0);
            m_BBfixed_CT_polar = m_BBfixed_CT_polarRect.block(0,0, N_POLAR, 7);
            m_BBfixed_CT_rectangular = m_BBfixed_CT_polarRect.block(N_POLAR,0, N_RECT, 7);
//        }
//        if(mConcurrent2.resultCount() > 0)
//        {
            m_BBfixed_BB_polarRect = mConcurrent2.result();
            mConcurrent2 = QtConcurrent::run(this, &CyclicModel::cycle_recalculate_concurrentM, m_BBfixed_BB_filtered, m_omega0);
            m_BBfixed_BB_polar = m_BBfixed_BB_polarRect.block(0,0, N_POLAR, 7);
            m_BBfixed_BB_rectangular = m_BBfixed_BB_polarRect.block(N_POLAR,0, N_RECT, 7);
//        }
//        if(mConcurrent3.resultCount() > 0)
//        {
            m_Bird4_polarRect = mConcurrent3.result();
            mConcurrent3 = QtConcurrent::run(this, &CyclicModel::cycle_recalculate_concurrentV, m_Bird4_filtered_new, m_omega0); // m_Bird4_filtered
            m_Bird4_polar      = m_Bird4_polarRect.segment(0, N_POLAR);
            m_Bird4_rectangular      = m_Bird4_polarRect.segment(N_POLAR, N_RECT);
        }
        else
        {
            if( (!mConcurrent1.isRunning()) && (!mConcurrent2.isRunning()) && (!mConcurrent3.isRunning()) )
            {
                mConcurrent1 = QtConcurrent::run(this, &CyclicModel::cycle_recalculate_concurrentM, m_BBfixed_CT_filtered, m_omega0);
                mConcurrent2 = QtConcurrent::run(this, &CyclicModel::cycle_recalculate_concurrentM, m_BBfixed_BB_filtered, m_omega0);
                mConcurrent3 = QtConcurrent::run(this, &CyclicModel::cycle_recalculate_concurrentV, m_Bird4_filtered_new, m_omega0); // m_Bird4_filtered
//                std::cout << "Start concurrent." << std::endl;
            }

//            // TODO : is this the best way to test this? what if one of them finishes during these checks?
//            if( (!mConcurrent1.isRunning()) ) // not running
//                mConcurrent1 = QtConcurrent::run(this, &CyclicModel::cycle_recalculate_concurrentM, m_BBfixed_CT_filtered, m_omega0);
//            if( !mConcurrent2.isRunning()) // not running
//                mConcurrent2 = QtConcurrent::run(this, &CyclicModel::cycle_recalculate_concurrentM, m_BBfixed_BB_filtered, m_omega0);
//            if( !mConcurrent3.isRunning()) // not running
//                mConcurrent3 = QtConcurrent::run(this, &CyclicModel::cycle_recalculate_concurrentV, m_Bird4_filtered, m_omega0);
        }


        elNsec = elTimer.nsecsElapsed();
        std::cout << "Cycle recalc x2+x1 Nsec elapsed: " << elNsec << std::endl;
        elTimer.restart();

        // get predictions based on the models
        double t0 = m_timeData_new.back();
        double t1 = t0; // + 0.0; fudge factor may be needed?
        double t2 = (m_nFutureSamples - EDGE_EFFECT)*SAMPLE_DELTA_TIME;
        getPrediction7Axis(t1, m_BBfixed_CT_polar, m_BBfixed_CT_rectangular, m_BBfixed_CT_des);
        getPrediction7Axis(t2, m_BBfixed_CT_polar, m_BBfixed_CT_rectangular, m_BBfixed_CTtraj_future_des);
        getPrediction7Axis(t1, m_BBfixed_BB_polar, m_BBfixed_BB_rectangular, m_BBfixed_BB_des);


        elNsec = elTimer.nsecsElapsed();
        std::cout << "getPrediction Nsec elapsed: " << elNsec << std::endl;
        elTimer.restart();
    }
}

void CyclicModel::updatePeriod(const double period)
{
    // omega_0 = 2*pi*1/period; % 2*pi*breathing frequency (rad/sec)

    if(period > 20.)
        m_omega0 = 2.0*pi/BREATH_RATE;
    else
        m_omega0 = 2.0*pi/period;
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
            x_des += x_polar(i+1) * sin( (i+1.0)*x_polar(N_HARMONICS+1)*timeShift + atan2( x_rect(i+N_HARMONICS+1), x_rect(i+1) ) );
        }
        x_des += x_polar(0);
    }
    else
    {
        x_des = std::numeric_limits<double>::quiet_NaN();
    }

    return x_des ;
}

void CyclicModel::getPrediction7Axis(const double timeShift, const EigenMatrixPolar &x_polar, const EigenMatrixRectangular &x_rect, EigenVector7d &X_des)
{
    // Make sure the time coming in is already (t1[j] - t_begin)

    if(m_isTrained)
    {
        X_des = EigenVector7d::Zero();

        if(X_des.size() != 7)
            printf("X_des.size() != 7\n");

        for(size_t j = 0; j < X_des.size(); j++)
        {
            for (size_t i = 0; i < N_HARMONICS; i++)
            {
                X_des(j) = X_des(j) + x_polar(i+1,j) * sin( (i+1.0)*x_polar(N_HARMONICS+1,j)*timeShift + atan2( x_rect(i+N_HARMONICS+1,j), x_rect(i+1,j) ) );
            }
            X_des(j) = X_des(j) + x_polar(0,j);
        }
    }
    else
    {
        //X_des = EigenVector7d::Zero();
        X_des = EigenVector7d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
}

void CyclicModel::setInVivo(const bool isInVivo)
{
    if( (!m_isTrained) && (m_numSamples == 0) )
        m_isInVivo = isInVivo;
    else
        printf("Can't modify mode now!\n");
}

void CyclicModel::filterTrainingData()
{
    m_LowPassFilter.run(m_BBfixed_CT, m_BBfixed_CT_filtered);
    //m_LowPassFilter.run(m_BBfixed_Instr, m_BBfixed_Instr_filtered);
    m_LowPassFilter.run(m_BBfixed_BB, m_BBfixed_BB_filtered);
    m_LowPassFilter.run(m_Bird4, m_Bird4_filtered);
}

void CyclicModel::filterNewObservations()
{
    m_LowPassFilter.run(m_Bird4_new, m_Bird4_filtered_new);
}

double CyclicModel::peakDetector(const bool runForInit)
{
    std::vector<size_t> peakIdx; // container for indices
    double thresh;
    if(runForInit){
        // look at Bird4
        // EigenVectorFiltered::Index maxRow, maxCol, minRow, minCol;
        auto maxBird = m_Bird4_filtered.maxCoeff();//(&maxRow, &maxCol);
        auto minBird = m_Bird4_filtered.minCoeff();
        double diffBird = maxBird - minBird;
        thresh = minBird + diffBird * PEAK_THRESHOLD;

        // find indices that are greater than thresh

        auto it = std::find_if(m_Bird4_filtered.data(), m_Bird4_filtered.data() + m_Bird4_filtered.size(),
                               [thresh](double i){return i > thresh;});
        while (it != (m_Bird4_filtered.data() + m_Bird4_filtered.size()) ) {
           peakIdx.emplace_back(std::distance(m_Bird4_filtered.data(), it));
           it = std::find_if(std::next(it), m_Bird4_filtered.data() + m_Bird4_filtered.size(),
                             [thresh](double i){return i > thresh;});
        }
    }
    else
    {
        // look at Bird4
        // EigenVectorFiltered::Index maxRow, maxCol, minRow, minCol;
        auto maxBird = m_Bird4_filtered_new.maxCoeff();//(&maxRow, &maxCol);
        auto minBird = m_Bird4_filtered_new.minCoeff();
        double diffBird = maxBird - minBird;
        thresh = minBird + diffBird * PEAK_THRESHOLD;

        // find indices that are greater than thresh

        auto it = std::find_if(m_Bird4_filtered_new.data(), m_Bird4_filtered_new.data() + m_Bird4_filtered_new.size(),
                               [thresh](double i){return i > thresh;});
        while (it != (m_Bird4_filtered_new.data() + m_Bird4_filtered_new.size()) ) {
           peakIdx.emplace_back(std::distance(m_Bird4_filtered_new.data(), it));
           it = std::find_if(std::next(it), m_Bird4_filtered_new.data() + m_Bird4_filtered_new.size(),
                             [thresh](double i){return i > thresh;});
        }
    }

    // get the timestamp at peaks
    std::vector<double> peaksTime; peaksTime.reserve(peakIdx.size());
    // if initializing, then use "m_timeData_init", otherwise use "m_timeData_new"
    if(runForInit){
        for(auto it = peakIdx.begin(); it != peakIdx.end(); ++it){
            peaksTime.emplace_back(m_timeData_init[*it]);
        }
        std::cout << "///runForInit///" << std::endl;
    }
    else{
        for(auto it = peakIdx.begin(); it != peakIdx.end(); ++it){
            peaksTime.emplace_back(m_timeData_new[*it]);
        }
        std::cout << "///m_timeData_new///" << std::endl;
    }
    std::cout << std::endl;

    // take the difference of time stamps
    std::vector<double> peaksTdiff; peaksTdiff.reserve(peaksTime.size() - 1);
    std::transform(peaksTime.begin()+1,peaksTime.end(),
                   peaksTime.begin(),std::back_inserter(peaksTdiff),std::minus<double>());

    // find time differences larger than 1 seconds
    std::cout << "peakDetector peakGapsIdx: " << std::endl;
    double largeTimeGap = 1.0; // seconds
    std::vector<size_t> peakGapsIdx; // container for indices
    std::vector<double>::iterator it2 = std::find_if(peaksTdiff.begin(), peaksTdiff.end(),
                                                     [largeTimeGap](double i){return i > largeTimeGap;});
    while (it2 != peaksTdiff.end()) {
       peakGapsIdx.emplace_back(std::distance(peaksTdiff.begin(), it2));
       it2 = std::find_if(std::next(it2), peaksTdiff.end(),
                          [largeTimeGap](double i){return i > largeTimeGap;});
       std::cout << peakGapsIdx.back() << " ";
    }
    std::cout << std::endl;

    // we better have peaks
    if(peakGapsIdx.size() < 1)
    {
        std::cout << "PeakDetector is in trouble! n=" << peakGapsIdx.size() << std::endl;
        return BREATH_RATE;
    }

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
    if(runForInit)
    {
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
    }
    else{
        if( m_Bird4_filtered_new.head(1)[0] > thresh )
        {
            peakTimesRight.erase(peakTimesRight.begin());
            peakTimesLeft.erase(peakTimesLeft.begin());
        }
        if( m_Bird4_filtered_new.tail(1)[0] > thresh )
        {
            peakTimesRight.pop_back();
            peakTimesLeft.pop_back();
        }
    }

    // better not be empty
    if(peakTimesLeft.size() < 1)
    {
        std::cout << "PeakDetector is in trouble 2!\n" << std::endl;
        return BREATH_RATE;
    }

    // tops_peak_times_means = mean([tops_peak_times_right tops_peak_times_left],2);
    std::vector<double> mean; mean.reserve(peakTimesRight.size());
    std::transform(peakTimesRight.begin(), peakTimesRight.end(),
                   peakTimesLeft.begin(), std::back_inserter(mean),
                   [](double r, double l){ return (l+r)/2.0; });

//    if length(tops_peak_times_means)>1
//        tops_peak_times_diffs = tops_peak_times_means(2:end)-tops_peak_times_means(1:(end-1));
//        period=mean(tops_peak_times_diffs);
//    else
//        period=breath_expected;
//    end
    double period = 0.0;

    if(mean.size() > 1)
    {
        // take the difference of the means
        std::vector<double> meanDiffs; meanDiffs.reserve(mean.size());
        std::transform(mean.begin()+1, mean.end(),
                       mean.begin(), std::back_inserter(meanDiffs), std::minus<double>());

        period = std::accumulate(meanDiffs.begin(),meanDiffs.end(),0.0) / (double)meanDiffs.size();
    }
    else
    {
        period = BREATH_RATE;
    }

    if(runForInit)
    {
        // save the original peaks
        //m_respPeakMean = mean;

        // In vivo version has an error checker to make sure the breathing period is
        // between 4 - 5.5 seconds
        if(m_isInVivo)
        {   // clamp
            if( (period < (BREATH_RATE*0.9)) || (period > (BREATH_RATE*1.1)) )
                period = BREATH_RATE;
        }
    }
    else
    {
        double period_old = 2.0*pi/m_omega0;

        // Get the time difference between peaks
//        % All of these steps account for the fact that there could be multiple
//        % peaks detected for the model or for the measured values. We need to find
//        % the smallest absolute value peak time difference, but then preserve the
//        % sign on that time difference to know whether the period should be faster
//        % or slower.
//        for j=1:size(tops_peak_times_means_model,1)
//            for i=1:size(tops_peak_times_means_meas,1)
//                peak_time_diff(i,j) = tops_peak_times_means_model(j)-tops_peak_times_means_meas(i);
//            end
//            [~, idx(j)]=min(abs(peak_time_diff(:,j)));
//            time_diff(j)=peak_time_diff(idx(j),j);
//        end
//        [~,idx2]=min(abs(time_diff));
//        min_time_diff=time_diff(idx2);

        peakDetectorForBreathModel(); // filter the breathing signal
        m_respPeakMean = m_breathSignalPeakMean;

        if(m_respPeakMean.size() < 1)
        {
            std::cout << "PeakDetector is in trouble 3!\n" << std::endl;
            return period_old;
        }

        Eigen::MatrixXd peakTimeDiff = Eigen::MatrixXd::Zero(m_respPeakMean.size(), mean.size());
        Eigen::VectorXd timeDiff = Eigen::VectorXd::Zero(mean.size());
        double minTimeDiff;

        for(size_t iMeanModel = 0; iMeanModel < m_respPeakMean.size(); iMeanModel++)
        {
            for(size_t jMeanMeas = 0; jMeanMeas < mean.size(); jMeanMeas++)
            {
                peakTimeDiff(iMeanModel,jMeanMeas) = m_respPeakMean[iMeanModel] - mean[jMeanMeas];
            }
            Eigen::VectorXd::Index maxIdx;
            peakTimeDiff.row(iMeanModel).cwiseAbs().maxCoeff(&maxIdx);
            timeDiff[iMeanModel] = peakTimeDiff(iMeanModel,maxIdx);
        }
        Eigen::VectorXd::Index minIdx;
        timeDiff.cwiseAbs().minCoeff(&minIdx);
        minTimeDiff = timeDiff[minIdx];

        double period_new = period_old*(1.0 - minTimeDiff/(mean[0]-m_timeData_new[0])); // m_timeData_old[0]

        if(m_isInVivo) // in vivo mode
        {
            if( (period_new < (BREATH_RATE*0.9)) || (period > (BREATH_RATE*1.1)) )
                period_new = period_old;
        }
        else // benchtop mode
        {
            if( (period_new < (period_old*0.9)) || (period > (period_old*1.1)) )
                period_new = period_old;
        }

        period = period_new;
    }

    std::cout << "Period: " << period << std::endl;

    return period;
}

void CyclicModel::peakDetectorForBreathModel()
{
    // look at m_breathSignalFromModel
    auto maxBird = m_breathSignalFromModel.maxCoeff();
    auto minBird = m_breathSignalFromModel.minCoeff();
    double diffBird = maxBird - minBird;
    double thresh = minBird + diffBird * PEAK_THRESHOLD;

    // find indices that are greater than thresh
    std::vector<size_t> peakIdx; // container for indices
    auto it = std::find_if(m_breathSignalFromModel.data(), m_breathSignalFromModel.data() + m_breathSignalFromModel.size(),
                           [thresh](double i){return i > thresh;});
    while (it != (m_breathSignalFromModel.data() + m_breathSignalFromModel.size()) ) {
       peakIdx.emplace_back(std::distance(m_breathSignalFromModel.data(), it));
       it = std::find_if(std::next(it), m_breathSignalFromModel.data() + m_breathSignalFromModel.size(),
                         [thresh](double i){return i > thresh;});
    }

    // get the timestamp at peaks

    std::vector<double> peaksTime; peaksTime.reserve(peakIdx.size());
    // read m_timeData_new
    for(auto it = peakIdx.begin(); it != peakIdx.end(); ++it){
        peaksTime.emplace_back(m_timeData_new[*it + EDGE_EFFECT]);
    }

    // take the difference of time stamps
    std::vector<double> peaksTdiff; peaksTdiff.reserve(peaksTime.size() - 1);
    std::transform(peaksTime.begin()+1,peaksTime.end(),
                   peaksTime.begin(),std::back_inserter(peaksTdiff),std::minus<double>());


    // find time differences larger than 1 seconds
    std::cout << "peakDetectorForBreathModel : peakGapsIdx: " << std::endl;
    double largeTimeGap = 1.0; // seconds
    std::vector<size_t> peakGapsIdx; // container for indices
    auto it2 = std::find_if(std::begin(peaksTdiff), std::end(peaksTdiff),
                            [largeTimeGap](double i){return i > largeTimeGap;});
    while (it2 != std::end(peaksTdiff)) {
       peakGapsIdx.emplace_back(std::distance(std::begin(peaksTdiff), it2));
       it2 = std::find_if(std::next(it2), std::end(peaksTdiff),
                          [largeTimeGap](double i){return i > largeTimeGap;});
       std::cout << peakGapsIdx.back() << " ";
    }
    std::cout << std::endl;


    // we better have peaks
    if(peakGapsIdx.size() < 1)
    {
        std::cout << "PeakDetector is in trouble! n=" << peakGapsIdx.size() << std::endl;
        return;
    }

    std::vector<double> peakTimesRight, peakTimesLeft;
    peakTimesLeft.emplace_back(peaksTime.front());
    for(auto it3 = peakGapsIdx.begin(); it3 != peakGapsIdx.end(); ++it3){
        peakTimesRight.emplace_back(peaksTime[*it3]);
        peakTimesLeft.emplace_back(peaksTime[1 + *it3]);
    }
    peakTimesRight.emplace_back(peaksTime.back());

    if( m_breathSignalFromModel.head(1)[0] > thresh )
    {
        peakTimesRight.erase(peakTimesRight.begin());
        peakTimesLeft.erase(peakTimesLeft.begin());
    }
    if( m_breathSignalFromModel.tail(1)[0] > thresh )
    {
        peakTimesRight.pop_back();
        peakTimesLeft.pop_back();
    }

    // better not be empty
    if(peakTimesLeft.size() < 1)
    {
        std::cout << "PeakDetector is in trouble 2! n=" << peakTimesLeft.size() << std::endl;
        return;
    }

    // tops_peak_times_means = mean([tops_peak_times_right tops_peak_times_left],2);
    std::vector<double> mean; mean.reserve(peakTimesRight.size());
    std::transform(peakTimesRight.begin(), peakTimesRight.end(),
                   peakTimesLeft.begin(), std::back_inserter(mean),
                   [](double r, double l){ return (l+r)/2.0; });

    m_breathSignalPeakMean = mean;
}

void CyclicModel::cycle_recalculate(const EigenMatrixFiltered &z_init,
                                    EigenMatrixRectangular &x_rect,
                                    EigenMatrixPolar &x_polar)
{
    // TODO: add error checking to ensure that the vector size is correct
    if( z_init.cols() != 7 )
        printf("cycle_recalculate: Wrong column size!\n");
    if( z_init.rows() != N_FILTERED )
        printf("cycle_recalculate: Wrong row size!\n");

    // get all of the constants
    size_t m = N_HARMONICS; // m = number of sinusoid components
    double delta_t = SAMPLE_DELTA_TIME; // Time step for each collected data point
    size_t N_initpts = N_SAMPLES;  // number of initialization points
    size_t edge_effect = EDGE_EFFECT;
    double omega_0 = m_omega0;

    // calculate things from inputs
    size_t num_states = m * 2 + 2;
    //size_t N_filtered = N_initpts - 2 * edge_effect;

    // parse the already low pass filtered data
    Eigen::VectorXd z_init_x = z_init.col(0);
    Eigen::VectorXd z_init_y = z_init.col(1);
    Eigen::VectorXd z_init_z = z_init.col(2);
    Eigen::VectorXd z_init_xaxis = z_init.col(3);
    Eigen::VectorXd z_init_yaxis = z_init.col(4);
    Eigen::VectorXd z_init_zaxis = z_init.col(5);
    Eigen::VectorXd z_init_angle = z_init.col(6);

    // allocate zero matrices
    Eigen::MatrixXd A_init_x(N_FILTERED, num_states - 1); A_init_x.setZero(); A_init_x.col(0).setOnes();
    Eigen::MatrixXd A_init_y(N_FILTERED, num_states - 1); A_init_y.setZero(); A_init_y.col(0).setOnes();
    Eigen::MatrixXd A_init_z(N_FILTERED, num_states - 1); A_init_z.setZero(); A_init_z.col(0).setOnes();
    Eigen::MatrixXd A_init_xaxis(N_FILTERED, num_states - 1); A_init_xaxis.setZero(); A_init_xaxis.col(0).setOnes();
    Eigen::MatrixXd A_init_yaxis(N_FILTERED, num_states - 1); A_init_yaxis.setZero(); A_init_yaxis.col(0).setOnes();
    Eigen::MatrixXd A_init_zaxis(N_FILTERED, num_states - 1); A_init_zaxis.setZero(); A_init_zaxis.col(0).setOnes();
    Eigen::MatrixXd A_init_angle(N_FILTERED, num_states - 1); A_init_angle.setZero(); A_init_angle.col(0).setOnes();

    for (int i = 0; i < N_FILTERED; i++)
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

    x_rect.col(0) = x_init_x;
    x_rect.col(1) = x_init_y;
    x_rect.col(2) = x_init_z;
    x_rect.col(3) = x_init_xaxis;
    x_rect.col(4) = x_init_yaxis;
    x_rect.col(5) = x_init_zaxis;
    x_rect.col(6) = x_init_angle;

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

    x_polar.col(0) = x_polar_x;
    x_polar.col(1) = x_polar_y;
    x_polar.col(2) = x_polar_z;
    x_polar.col(3) = x_polar_xaxis;
    x_polar.col(4) = x_polar_yaxis;
    x_polar.col(5) = x_polar_zaxis;
    x_polar.col(6) = x_polar_angle;
}

void CyclicModel::cycle_recalculate(const EigenVectorFiltered &z_init, EigenVectorRectangular &x_rect, EigenVectorPolar &x_polar)
{
    // TODO: add error checking to ensure that the vector size is correct
    if( z_init.rows() != N_FILTERED )
        printf("cycle_recalculate: Wrong row size!\n");

    // get all of the constants
    size_t m = N_HARMONICS; // m = number of sinusoid components
    double delta_t = SAMPLE_DELTA_TIME; // Time step for each collected data point
    size_t N_initpts = N_SAMPLES;  // number of initialization points
    size_t edge_effect = EDGE_EFFECT;
    double omega_0 = m_omega0;

    // calculate things from inputs
    size_t num_states = m * 2 + 2;
    //size_t N_filtered = N_initpts - 2 * edge_effect;

    // parse the already low pass filtered data
    // Eigen::VectorXd z_init_x = z_init;

    // allocate zero matrices
    Eigen::MatrixXd A_init(N_FILTERED, num_states - 1); A_init.setZero(); A_init.col(0).setOnes();

    for (int i = 0; i < N_FILTERED; i++)
    {
        for (int j = 1; j <= m; j++)
        {
            A_init(i, j)			= sin(j*omega_0*i*delta_t);
            A_init(i, j + m)		= cos(j*omega_0*i*delta_t);
        }
    }

    // Step 2. Use least squares estimate to solve for x.
    //VectorXd x_init_x = pseudoInverse(A_init) * z_init;
    Eigen::VectorXd x_init = A_init.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init);

    x_rect = x_init;

    // Step 3. Convert from rectangular into polar and assemble the state
    // vector, x, at k = 430 (which is the last trustworthy state we can know)
    // State vector, x
    // x = [c; r(1:4); omega; theta(1:4)];

    x_polar.setZero(num_states);
    x_polar(0) = x_init(0); // c(dc offset)

    for (size_t i = 1; i <= m; i++)
    {
        x_polar(i) = std::sqrt(std::pow(x_init(i), 2) + std::pow(x_init(i + m), 2)); // r_i, convert rectangular coords back to polar
    }

    x_polar(m + 1) = omega_0; // omega_0, (rad / sec)

    for (int i = 0; i < m; i++)
    {
        x_polar(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init(i + m + 1), x_init(i + 1)); // theta = i*omega*T + phi, (rad)
    }

    //x_polar = x_polar;
}

Eigen::MatrixXd CyclicModel::cycle_recalculate_concurrentM(const EigenMatrixFiltered &z_init, const double omega0)
{
    if( z_init.cols() != 7 )
        printf("cycle_recalculate: Wrong column size!\n");
    if( z_init.rows() != N_FILTERED )
        printf("cycle_recalculate: Wrong row size!\n");

    // get all of the constants
    size_t m = N_HARMONICS; // m = number of sinusoid components
    double delta_t = SAMPLE_DELTA_TIME; // Time step for each collected data point
    size_t N_initpts = N_SAMPLES;  // number of initialization points
    //size_t edge_effect = EDGE_EFFECT;
    double omega_0 = omega0;

    // calculate things from inputs
    size_t num_states = m * 2 + 2;
    //size_t N_filtered = N_initpts - 2 * edge_effect;

    // parse the already low pass filtered data
    Eigen::VectorXd z_init_x = z_init.col(0);
    Eigen::VectorXd z_init_y = z_init.col(1);
    Eigen::VectorXd z_init_z = z_init.col(2);
    Eigen::VectorXd z_init_xaxis = z_init.col(3);
    Eigen::VectorXd z_init_yaxis = z_init.col(4);
    Eigen::VectorXd z_init_zaxis = z_init.col(5);
    Eigen::VectorXd z_init_angle = z_init.col(6);

    // allocate zero matrices
    Eigen::MatrixXd A_init_x(N_FILTERED, num_states - 1); A_init_x.setZero(); A_init_x.col(0).setOnes();
    Eigen::MatrixXd A_init_y(N_FILTERED, num_states - 1); A_init_y.setZero(); A_init_y.col(0).setOnes();
    Eigen::MatrixXd A_init_z(N_FILTERED, num_states - 1); A_init_z.setZero(); A_init_z.col(0).setOnes();
    Eigen::MatrixXd A_init_xaxis(N_FILTERED, num_states - 1); A_init_xaxis.setZero(); A_init_xaxis.col(0).setOnes();
    Eigen::MatrixXd A_init_yaxis(N_FILTERED, num_states - 1); A_init_yaxis.setZero(); A_init_yaxis.col(0).setOnes();
    Eigen::MatrixXd A_init_zaxis(N_FILTERED, num_states - 1); A_init_zaxis.setZero(); A_init_zaxis.col(0).setOnes();
    Eigen::MatrixXd A_init_angle(N_FILTERED, num_states - 1); A_init_angle.setZero(); A_init_angle.col(0).setOnes();

    for (int i = 0; i < N_FILTERED; i++)
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

    EigenMatrixRectangular x_rect;
    x_rect.col(0) = x_init_x;
    x_rect.col(1) = x_init_y;
    x_rect.col(2) = x_init_z;
    x_rect.col(3) = x_init_xaxis;
    x_rect.col(4) = x_init_yaxis;
    x_rect.col(5) = x_init_zaxis;
    x_rect.col(6) = x_init_angle;

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

    EigenMatrixPolar x_polar;

    x_polar.col(0) = x_polar_x;
    x_polar.col(1) = x_polar_y;
    x_polar.col(2) = x_polar_z;
    x_polar.col(3) = x_polar_xaxis;
    x_polar.col(4) = x_polar_yaxis;
    x_polar.col(5) = x_polar_zaxis;
    x_polar.col(6) = x_polar_angle;

    Eigen::MatrixXd result(N_POLAR + N_RECT, 7);
    result << x_polar, x_rect;

    return result;
}

Eigen::VectorXd CyclicModel::cycle_recalculate_concurrentV(const EigenVectorFiltered &z_init, const double omega0)
{
    // TODO: add error checking to ensure that the vector size is correct
    if( z_init.rows() != N_FILTERED )
        printf("cycle_recalculate: Wrong row size!\n");

    EigenVectorPolar x_polar;
    EigenVectorRectangular x_rect;


    // get all of the constants
    size_t m = N_HARMONICS; // m = number of sinusoid components
    double delta_t = SAMPLE_DELTA_TIME; // Time step for each collected data point
    size_t N_initpts = N_SAMPLES;  // number of initialization points
    //size_t edge_effect = EDGE_EFFECT;
    double omega_0 = omega0;

    // calculate things from inputs
    size_t num_states = m * 2 + 2;
    //size_t N_filtered = N_initpts - 2 * edge_effect;

    // parse the already low pass filtered data
    // Eigen::VectorXd z_init_x = z_init;

    // allocate zero matrices
    Eigen::MatrixXd A_init(N_FILTERED, num_states - 1); A_init.setZero(); A_init.col(0).setOnes();

    for (int i = 0; i < N_FILTERED; i++)
    {
        for (int j = 1; j <= m; j++)
        {
            A_init(i, j)			= sin(j*omega_0*i*delta_t);
            A_init(i, j + m)		= cos(j*omega_0*i*delta_t);
        }
    }

    // Step 2. Use least squares estimate to solve for x.
    //VectorXd x_init_x = pseudoInverse(A_init) * z_init;
    Eigen::VectorXd x_init = A_init.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z_init);

    x_rect = x_init;

    // Step 3. Convert from rectangular into polar and assemble the state
    // vector, x, at k = 430 (which is the last trustworthy state we can know)
    // State vector, x
    // x = [c; r(1:4); omega; theta(1:4)];

    x_polar.setZero(num_states);
    x_polar(0) = x_init(0); // c(dc offset)

    for (size_t i = 1; i <= m; i++)
    {
        x_polar(i) = std::sqrt(std::pow(x_init(i), 2) + std::pow(x_init(i + m), 2)); // r_i, convert rectangular coords back to polar
    }

    x_polar(m + 1) = omega_0; // omega_0, (rad / sec)

    for (int i = 0; i < m; i++)
    {
        x_polar(i + m + 2) = (i + 1.0)*omega_0*N_initpts*delta_t + atan2(x_init(i + m + 1), x_init(i + 1)); // theta = i*omega*T + phi, (rad)
    }

    Eigen::VectorXd result(N_POLAR + N_RECT);;
    result << x_polar, x_rect;

    return result;
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
