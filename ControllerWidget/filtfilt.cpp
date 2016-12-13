#include "filtfilt.h"

filtfilt::filtfilt()
{
    m_A = {1};

    // From MATLAB : m_B = fir1(50,(100/60/2)/(150/2),'low'); // order 50, 100 bpm, 150 Hz sampling
//    m_B = {0.00264726249703924,           0.00279642405272151,           0.00319013679531822,           0.00382941384359819,
//           0.00471069197708524,           0.00582582662708399,           0.00716217671948781,           0.00870277868575645,
//           0.0104266071117473,            0.0123089176854006,            0.0143216663725929,            0.0164339971333124,
//           0.0186127890229487,            0.0208232522381201,            0.0230295615915197,            0.0251955150597567,
//           0.0272852044611377,            0.0292636850004373,            0.0310976303728649,            0.0327559603516814,
//           0.0342104282892947,            0.0354361567303369,            0.0364121103516330,            0.0371214966872074,
//           0.0375520865406890,            0.0376964476024575,            0.0375520865406890,            0.0371214966872074,
//           0.0364121103516330,            0.0354361567303369,            0.0342104282892947,            0.0327559603516814,
//           0.0310976303728649,            0.0292636850004373,            0.0272852044611377,            0.0251955150597567,
//           0.0230295615915197,            0.0208232522381201,            0.0186127890229487,            0.0164339971333124,
//           0.0143216663725929,            0.0123089176854006,            0.0104266071117473,            0.00870277868575645,
//           0.00716217671948781,           0.00582582662708399,           0.00471069197708524,           0.00382941384359819,
//           0.00319013679531822,           0.00279642405272151,           0.00264726249703924};

    // From MATLAB : m_B = fir1(50,(120/60/2)/(42.7350/2),'low'); // order 50, 100 bpm, 42.7350 Hz sampling
//    m_B = {-0.000569727095790538,       -0.000460005562187752,        -0.000341437553206024,        -0.000165785763108122,
//           0.000123143497281698,         0.000586219883270480,         0.00128516073718559,          0.00227837404637298,
//           0.00361672040102945,          0.00533945551423732,          0.00747061259853524,          0.0100160653579639,
//           0.0129614783838126,           0.0162713041615132,           0.0198889274205894,           0.0237379916329247,
//           0.0277248730484729,           0.0317421989856768,           0.0356732434065773,           0.0393969780822734,
//           0.0427935153555936,           0.0457496513584374,           0.0481642083490973,           0.0499528823717932,
//           0.0510523273876564,           0.0514232479879954,           0.0510523273876564,           0.0499528823717932,
//           0.0481642083490973,           0.0457496513584374,           0.0427935153555936,           0.0393969780822734,
//           0.0356732434065773,           0.0317421989856768,           0.0277248730484729,           0.0237379916329247,
//           0.0198889274205894,           0.0162713041615132,           0.0129614783838126,           0.0100160653579639,
//           0.00747061259853524,          0.00533945551423732,          0.00361672040102945,          0.00227837404637298,
//           0.00128516073718559,          0.000586219883270480,         0.000123143497281698,        -0.000165785763108122,
//           -0.000341437553206024,       -0.000460005562187752,        -0.000569727095790538};

    size_t filterOrder = FILTER_ORDER + 1; // must be odd!
    double deltaT = 0.0234; // = SAMPLE_DELTA_TIME;
    double samplingFreq = 1.0/deltaT;
    double band = (HEART_RATE/60./2.)/(samplingFreq/2.);

    m_zzi = std::allocate_shared< Eigen::MatrixXd >( Eigen::aligned_allocator<Eigen::MatrixXd>() );

    m_B = FIR_LPF_LeastSquares(filterOrder, band);
//    for(auto x : m_B)
//        std::cout << x << std::endl;

    updateFilterParameters();
}

filtfilt::~filtfilt()
{

}

void filtfilt::run(const std::vector<double> &X, std::vector<double> &Y)
{
    int len = X.size();     // length of input

    if (len <= m_nfact)
    {
        std::cerr << "Input data too short! Data must have length more than 3 times filter order." << std::endl;
        return;
    }

    std::vector<double> leftpad = subvector_reverse(X, m_nfact, 1);
    double _2x0 = 2 * X[0];
    std::transform(leftpad.begin(), leftpad.end(), leftpad.begin(), [_2x0](double val) {return _2x0 - val; });

    std::vector<double> rightpad = subvector_reverse(X, len - 2, len - m_nfact - 1);
    double _2xl = 2 * X[len-1];
    std::transform(rightpad.begin(), rightpad.end(), rightpad.begin(), [_2xl](double val) {return _2xl - val; });

    double y0;
    std::vector<double> signal1, signal2, zi;

    signal1.reserve(leftpad.size() + X.size() + rightpad.size());
    append_vector(signal1, leftpad);
    append_vector(signal1, X);
    append_vector(signal1, rightpad);

    zi.resize(m_zzi->size());

    // Do the forward and backward filtering
    y0 = signal1[0];
    std::transform(m_zzi->data(), m_zzi->data() + m_zzi->size(), zi.begin(), [y0](double val){ return val*y0; });
    filter(signal1, signal2, zi);
    std::reverse(signal2.begin(), signal2.end());
    y0 = signal2[0];
    std::transform(m_zzi->data(), m_zzi->data() + m_zzi->size(), zi.begin(), [y0](double val){ return val*y0; });
    filter(signal2, signal1, zi);
    Y = subvector_reverse(signal1, signal1.size() - m_nfact - 1, m_nfact);
}

void filtfilt::run(const std::vector<double> &X, EigenVectorFiltered &Y)
{
    int len = X.size();     // length of input

    if (len <= m_nfact)
    {
        std::cerr << "Input data too short! Data must have length more than 3 times filter order." << std::endl;
        return;
    }

    std::vector<double> leftpad = subvector_reverse(X, m_nfact, 1);
    double _2x0 = 2 * X[0];
    std::transform(leftpad.begin(), leftpad.end(), leftpad.begin(), [_2x0](double val) {return _2x0 - val; });

    std::vector<double> rightpad = subvector_reverse(X, len - 2, len - m_nfact - 1);
    double _2xl = 2 * X[len-1];
    std::transform(rightpad.begin(), rightpad.end(), rightpad.begin(), [_2xl](double val) {return _2xl - val; });

    double y0;
    std::vector<double> signal1, signal2, zi;

    signal1.reserve(leftpad.size() + X.size() + rightpad.size());
    append_vector(signal1, leftpad);
    append_vector(signal1, X);
    append_vector(signal1, rightpad);

    zi.resize(m_zzi->size());

    // Do the forward and backward filtering
    y0 = signal1[0];
    std::transform(m_zzi->data(), m_zzi->data() + m_zzi->size(), zi.begin(), [y0](double val){ return val*y0; });
    filter(signal1, signal2, zi);
    std::reverse(signal2.begin(), signal2.end());
    y0 = signal2[0];
    std::transform(m_zzi->data(), m_zzi->data() + m_zzi->size(), zi.begin(), [y0](double val){ return val*y0; });
    filter(signal2, signal1, zi);
    subvector_reverseEig(signal1, signal1.size() - m_nfact - 1 - EDGE_EFFECT, m_nfact + EDGE_EFFECT, Y);
}

void filtfilt::run(const EigenStdVecVector7d &X, EigenStdVecVector7d &Y)
{
    int len = X.size();     // length of input

    if (len <= m_nfact)
    {
        std::cerr << "Input data too short! Data must have length more than 3 times filter order." << std::endl;
        return;
    }

    EigenStdVecVector7d leftpad = subvector_reverseEig(X, m_nfact, 1);
    EigenVector7d _2x0 = X[0] * 2.0;
    std::transform(leftpad.begin(), leftpad.end(), leftpad.begin(), [_2x0](EigenVector7d val) {return _2x0 - val; });

    EigenStdVecVector7d rightpad = subvector_reverseEig(X, len - 2, len - m_nfact - 1);
    EigenVector7d _2xl = X[len-1] * 2.0;
    std::transform(rightpad.begin(), rightpad.end(), rightpad.begin(), [_2xl](EigenVector7d val) {return _2xl - val; });

    EigenVector7d y0;
    EigenStdVecVector7d signal1, signal2, zi;

    // signal1.reserve(leftpad.size() + X.size() + rightpad.size());
    append_vector(signal1, leftpad);
    append_vector(signal1, X);
    append_vector(signal1, rightpad);

    zi.resize(m_zzi->size());

    // Do the forward and backward filtering
    y0 = signal1[0];
    std::transform(m_zzi->data(), m_zzi->data() + m_zzi->size(), zi.begin(), [y0](double val){ return y0 * val; });
    filter(signal1, signal2, zi);
    std::reverse(signal2.begin(), signal2.end());
    y0 = signal2[0];
    std::transform(m_zzi->data(), m_zzi->data() + m_zzi->size(), zi.begin(), [y0](double val){ return y0 * val; });
    filter(signal2, signal1, zi);
    Y = subvector_reverseEig(signal1, signal1.size() - m_nfact - 1, m_nfact);
}

void filtfilt::run(const EigenStdVecVector7d &X, EigenMatrixFiltered &Y)
{
    int len = X.size();     // length of input , should be equal to N_INPUTS

    if (len <= m_nfact)
    {
        std::cerr << "Input data too short! Data must have length more than 3 times filter order." << std::endl;
        return;
    }

    EigenStdVecVector7d leftpad = subvector_reverseEig(X, m_nfact, 1);
    EigenVector7d _2x0 = X[0] * 2.0;
    std::transform(leftpad.begin(), leftpad.end(), leftpad.begin(), [_2x0](EigenVector7d val) {return _2x0 - val; });

    EigenStdVecVector7d rightpad = subvector_reverseEig(X, len - 2, len - m_nfact - 1);
    EigenVector7d _2xl = X[len-1] * 2.0;
    std::transform(rightpad.begin(), rightpad.end(), rightpad.begin(), [_2xl](EigenVector7d val) {return _2xl - val; });

    EigenVector7d y0;
    EigenStdVecVector7d signal1, signal2, zi;

    // signal1.reserve(leftpad.size() + X.size() + rightpad.size());
    append_vector(signal1, leftpad);
    append_vector(signal1, X);
    append_vector(signal1, rightpad);

    zi.resize(m_zzi->size());

    // Do the forward and backward filtering
    y0 = signal1[0];
    std::transform(m_zzi->data(), m_zzi->data() + m_zzi->size(), zi.begin(), [y0](double val){ return y0 * val; });
    filter(signal1, signal2, zi);
    std::reverse(signal2.begin(), signal2.end());
    y0 = signal2[0];
    std::transform(m_zzi->data(), m_zzi->data() + m_zzi->size(), zi.begin(), [y0](double val){ return y0 * val; });
    filter(signal2, signal1, zi);
    subvector_reverseEig(signal1, signal1.size() - m_nfact - EDGE_EFFECT - 1, m_nfact + EDGE_EFFECT, Y);
}

void filtfilt::filter(const std::vector<double> &X, std::vector<double> &Y, std::vector<double> &Zi)
{
    size_t input_size = X.size();
    //size_t filter_order = std::max(m_A.size(), m_B.size());
    size_t filter_order = m_nfilt;
//    m_B.resize(filter_order, 0);
//    m_A.resize(filter_order, 0);
    Zi.resize(filter_order, 0);
    Y.resize(input_size);

    const double *x = &X[0];
    const double *b = &m_B[0];
    const double *a = &m_A[0];
    double *z = &Zi[0];
    double *y = &Y[0];

    QElapsedTimer elTimer;
    elTimer.start();

    for (size_t i = 0; i < input_size; ++i)
    {
        size_t order = filter_order - 1;
        while (order)
        {
            if (i >= order)
            {
                z[order - 1] = b[order] * x[i - order] - a[order] * y[i - order] + z[order];
            }
            --order;
        }
        y[i] = b[0] * x[i] + z[0];
    }
    Zi.resize(filter_order - 1);

    qint64 elNsec = elTimer.nsecsElapsed();
    std::cout << "\nNsec elapsed:" << elNsec << std::endl;
}

void filtfilt::filter(const EigenStdVecVector7d &X, EigenStdVecVector7d &Y, EigenStdVecVector7d &Zi)
{
    size_t input_size = X.size();
    //size_t filter_order = std::max(m_A.size(), m_B.size());
    size_t filter_order = m_nfilt;
//    m_B.resize(filter_order, 0);
//    m_A.resize(filter_order, 0);
    Zi.resize(filter_order, EigenVector7d::Zero());
    Y.resize(input_size);

//    const EigenVector7d *x = &X[0];
    const double *b = &m_B[0];
    const double *a = &m_A[0];
//    EigenVector7d *z = &Zi[0];
//    EigenVector7d *y = &Y[0];

    QElapsedTimer elTimer;
    elTimer.start();

    for (size_t i = 0; i < input_size; ++i)
    {
        size_t order = filter_order - 1;
        // if(i < order)
        //    order = i; // may lead to tiny tiny speedup
        while (order)
        {
            if (i >= order)
            {
                Zi[order - 1] = (X[i - order] * b[order] - Y[i - order] * a[order] + Zi[order]).eval();
            }
            --order;
        }
        Y[i] = X[i] * b[0] + Zi[0];
    }
    Zi.resize(filter_order - 1);

    qint64 elNsec = elTimer.nsecsElapsed();
    std::cout << "\nNsec elapsed:" << elNsec << std::endl;
}

void filtfilt::setFilterCoefficients(const std::vector<double> &B, const std::vector<double> &A)
{
    m_A = A;
    m_B = B;

    updateFilterParameters();
}

void filtfilt::updateFilterParameters()
{
    m_na = m_A.size();
    m_nb = m_B.size();
    m_nfilt = (m_nb > m_na) ? m_nb : m_na;
    m_nfact = 3 * (m_nfilt - 1); // length of edge transients

    // set up filter's initial conditions to remove DC offset problems at the
    // beginning and end of the sequence
    m_B.resize(m_nfilt, 0);
    m_A.resize(m_nfilt, 0);

    m_rows.clear(); m_cols.clear();

    //rows = [1:nfilt-1           2:nfilt-1             1:nfilt-2];
    add_index_range(m_rows, 0, m_nfilt - 2);
    if (m_nfilt > 2)
    {
        add_index_range(m_rows, 1, m_nfilt - 2);
        add_index_range(m_rows, 0, m_nfilt - 3);
    }
    //cols = [ones(1,nfilt-1)         2:nfilt-1          2:nfilt-1];
    add_index_const(m_cols, 0, m_nfilt - 1);
    if (m_nfilt > 2)
    {
        add_index_range(m_cols, 1, m_nfilt - 2);
        add_index_range(m_cols, 1, m_nfilt - 2);
    }
    // data = [1+a(2)         a(3:nfilt)        ones(1,nfilt-2)    -ones(1,nfilt-2)];

    auto klen = m_rows.size();
    m_data.clear();
    m_data.resize(klen);
    m_data[0] = 1 + m_A[1];  int j = 1;
    if (m_nfilt > 2)
    {
        for (int i = 2; i < m_nfilt; i++)
            m_data[j++] = m_A[i];
        for (int i = 0; i < m_nfilt - 2; i++)
            m_data[j++] = 1.0;
        for (int i = 0; i < m_nfilt - 2; i++)
            m_data[j++] = -1.0;
    }

    // Calculate initial conditions
    Eigen::MatrixXd sp = Eigen::MatrixXd::Zero(max_val(m_rows) + 1, max_val(m_cols) + 1);
    for (size_t k = 0; k < klen; ++k)
    {
        sp(m_rows[k], m_cols[k]) = m_data[k];
    }
    auto bb = Eigen::VectorXd::Map(m_B.data(), m_B.size());
    auto aa = Eigen::VectorXd::Map(m_A.data(), m_A.size());
    (*m_zzi) = (sp.inverse() * (bb.segment(1, m_nfilt - 1) - (bb(0) * aa.segment(1, m_nfilt - 1))));



    if (m_A.empty())
    {
        std::cerr << "The feedback filter coefficients are empty." << std::endl;
    }
    if (std::all_of(m_A.begin(), m_A.end(), [](double coef){ return coef == 0; }))
    {
        std::cerr << "At least one of the feedback filter coefficients has to be non-zero." << std::endl;
    }
    if (m_A[0] == 0)
    {
        std::cerr << "First feedback coefficient has to be non-zero." << std::endl;
    }

    // Normalize feedback coefficients if a[0] != 1;
    auto a0 = m_A[0];
    if (a0 != 1.0)
    {
        std::transform(m_A.begin(), m_A.end(), m_A.begin(), [a0](double v) { return v / a0; });
        std::transform(m_B.begin(), m_B.end(), m_B.begin(), [a0](double v) { return v / a0; });
    }
}

void filtfilt::add_index_range(std::vector<int> &indices, int beg, int end, int inc)
{
    for (int i = beg; i <= end; i += inc)
    {
       indices.push_back(i);
    }
}

void filtfilt::add_index_const(std::vector<int> &indices, int value, size_t numel)
{
    while (numel--)
    {
        indices.push_back(value);
    }
}

void filtfilt::append_vector(std::vector<double> &vec, const std::vector<double> &tail)
{
    vec.insert(vec.end(), tail.begin(), tail.end());
}

void filtfilt::append_vector(EigenStdVecVector7d &vec, const EigenStdVecVector7d &tail)
{
    vec.insert(vec.end(), tail.begin(), tail.end());
}

std::vector<double> filtfilt::subvector_reverse(const std::vector<double> &vec, int idx_end, int idx_start)
{
    std::vector<double> result(&vec[idx_start], &vec[idx_end+1]);
    std::reverse(result.begin(), result.end());
    return result;
}

EigenStdVecVector7d filtfilt::subvector_reverseEig(const EigenStdVecVector7d &vec, int idx_end, int idx_start)
{
    EigenStdVecVector7d result(&vec[idx_start], &vec[idx_end+1]);
    std::reverse(result.begin(), result.end());
    return result;
}

void filtfilt::subvector_reverseEig(const std::vector<double> &vec, int idx_end, int idx_start,
                                    EigenVectorFiltered &Y)
{
    Eigen::Map<const EigenVectorFiltered> temp(&vec[idx_start], idx_end - idx_start + 1);
    Y = temp.reverse();
}

void filtfilt::subvector_reverseEig(const EigenStdVecVector7d &vec, int idx_end, int idx_start,
                                    EigenMatrixFiltered &Y)
{
    Y = EigenMatrixFiltered::Zero(idx_end - idx_start + 1, 7);

    for(size_t i = 0; i < (idx_end - idx_start + 1); i++)
    {
        Y.row(i) = vec[idx_end - i];
    }

}

// L should be odd
std::vector<double> filtfilt::FIR_LPF_LeastSquares(size_t L, const double deltaT)
{
    if(deltaT > 1.0)
        std::cerr << "deltaT is too large, sample faster!!!" << std::endl;

    std::vector<double> F = {0.0, deltaT/2.0, deltaT/2.0, 0.5},
                        dF = {deltaT/2.0, 0.0, 0.5 - deltaT/2.0},
                        M = {1,1,0,0},
                        W = {1,1};

    // size_t fullBand = 1, constantWeights = 1;

    size_t N = L;
    L = (N-1)/2;

    //  k=m=(0:L); // Type-I
    double b0 = 0.0;
    std::vector<double> b(L, 0.0);
    std::vector<size_t> m(L+1), k(L);

    for(size_t i = 0; i < m.size(); i++)
        m[i] = i;
    for(size_t i = 0; i < k.size(); i++)
        k[i] = m[i+1];

    for(size_t s = 0; s < 3; s += 2)
    {
        // m=(M(s+1)-M(s))/(F(s+1)-F(s));    %  slope
        // b1=M(s)-m*F(s);                   %  y-intercept

        //double m_ = (M[s+1] - M[s])/(F[s+1] - F[s]);
        double b1 = M[s];// - m_*F[s];

        //b0 = b0 + (b1*(F(s+1)-F(s)) + m/2*(F(s+1)*F(s+1)-F(s)*F(s)))* abs(W((s+1)/2)^2) ;
        //b0 += ( b1*(F[s+1]-F[s]) + m_/2.0*(F[s+1]*F[s+1]-F[s]*F[s]) );//* pow(W[(s+1)/2],2); W = 1 anyways
        b0 += b1*(F[s+1]-F[s]);

        //b = b+(m/(4*pi*pi)*(cos(2*pi*k*F(s+1))-cos(2*pi*k*F(s)))./(k.*k))...
        //            * abs(W((s+1)/2)^2);
        //b = b + (F(s+1)*(m*F(s+1)+b1)*sinc(2*k*F(s+1)) ...
        //    - F(s)*(m*F(s)+b1)*sinc(2*k*F(s))) ...
        //    * abs(W((s+1)/2)^2);
        // sinc(x) = sin(pi*x)/(pi*x); ->> (sin(pi*2.0*k[i]*F[s+1])/(pi*2.0*k[i]*F[s+1]))

        for(size_t i = 0; i < b.size(); i++)
        {
            double sinc_ = boost::math::sinc_pi(pi*2.0*k[i]*F[s+1]);
            //b[i] += (m_/(4.0*pi*pi)*(cos(2.0*pi*k[i]*F[s+1])-cos(2.0*pi*k[i]*F[s]))/(1.0*k[i]*k[i]));//* pow(W[(s+1)/2],2);

            //b[i] += (F[s+1]*(m_*F[s+1]+b1)*sinc_ - F[s]*(m_*F[s]+b1)*sinc_);// * pow(W[(s+1)/2],2);
            b[i] += (F[s+1]*b1*sinc_ - F[s]*b1*sinc_);
        }
    }

    b.insert(b.begin(),b0);

    // a=(W(1)^2)*4*b;
    std::vector<double> a(b.size());
    for(size_t i = 0; i < a.size(); i++)
    {
        //a[i] = 4.0*b[i];
        a[i] = 2.0*b[i];
    }
    //a[0] /= 2.0;

    // h=[a(L+1:-1:2)/2; a(1); a(2:L+1)/2].';
    std::vector<double> h;// size N
    h.insert(h.end(), a.rbegin(), a.rend());
    h.insert(h.end(), a.begin()+1, a.end());

    // return h;


//    b = hh.*Wind(:)';
//    a = 1;

    // generate the Hamming window coefficients
    std::vector<double> Wind = genHammingWindow(N);

    // reuse b
    b.clear(); b.resize(N);

    for(size_t i = 0; i < b.size(); i++)
    {
        b[i] = h[i] * Wind[i];
    }

    // scale filter
    // b = b / sum(b);
    double sumb = std::accumulate(b.begin(),b.end(),0.0);

    for(size_t i = 0; i < b.size(); i++)
    {
        b[i] /= sumb;
    }

    return b;
}

std::vector<double> filtfilt::genHammingWindow(const size_t L)
{
    std::vector<double> w;

    if(L % 2 > 0)
    {        
        // Odd length window
        size_t half = (L+1)/2;
        w = calcWindow(half, L);
    }
    else
    {
        // Even length window
        size_t half = L/2;
        w = calcWindow(half, L);
    }

    return w;
}

std::vector<double> filtfilt::calcWindow(size_t m, size_t n)
{
    // x = (0:m-1)'/(n-1);
    double x = 0.0;
    std::vector<double> w;

    w.resize(m);

    // compute coefficients of the Hamming windows
    for(size_t i = 0; i < m; i++)
    {
        x = i/(n-1.0); // between 0.00 and 0.50
        w[i] = 0.54 - 0.46*cos(2*pi*x);
    }

    // w = [w; w(end-1:-1:1)]; // make symmetric
    w.insert(w.end(), w.rbegin()+1, w.rend());

    return w;
}
