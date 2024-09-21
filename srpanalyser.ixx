module;
#include<functional>
#include<algorithm>
export module srpanalyzer;

import periodanalizer;
import buffers;

namespace srp_analyzer
{
    using namespace trend_analizer;

    template<numeric T>
    class elalon final
    {
    public:

        void save_sample(T value) noexcept
        {
            period_ = value;
            valid_ = true;
        }

        T get_period() const noexcept { return period_; }
        bool valid() const noexcept { return valid_; }

    private:
        T period_{};
        bool valid_{ false };
    };

    template<numeric T>
    struct notch final
    {
        constexpr static unsigned short invalid = 0xFFFF;

        bool is_valid() const noexcept { return id != invalid; }
        bool set_invalid() noexcept { id = invalid; }

        unsigned short id{ invalid };
        T time{};
    };

    template<numeric T>
    class notches final
    {
        using notch_type = notch<T>;
        using cit_notches = std::vector<notch_type>::const_iterator;

        constexpr static int buffer_capacity = 10;

    public:
        notches()
            : it_notches_{ notches_.begin() }
            , it_notches_end_{ notches_.end() }
        { 
            notches_.reserve(buffer_capacity);
            raw_notches_.reserve(buffer_capacity);
        }

        void start_period(T start_time) noexcept
        {
            start_time_ = start_time;
            it_notches_ = notches_.begin();
            it_notches_end_ = notches_.end();
            if (raw_notches_.size() > 0)
                raw_notches_.clear();
        }

        notch_type check_next(T time) 
        {
            if (it_notches_ == it_notches_end_)
                return notch_type();
            if (time - start_time_ < (*it_notches_).time)
                return notch_type();
            return *it_notches_++;
        }

        bool add_new(unsigned short id, T time)
        {
            raw_notches_.emplace_back(id, time - start_time_);
            return true;
        }

        void cancel_period() noexcept
        {
            raw_notches_.clear();
        }

        void remove(unsigned short id)
        {
            auto new_end = std::remove(notches_.begin(), notches_.end(),
                [](notch_type n) { return n.id == id; });
            notches_.erase(new_end);
        }

        void add_new_notches(double correction)
        {
            if (raw_notches_.empty())
                return;
            for (auto&& n : raw_notches_) {
                auto it_end = notches_.end();
                auto it = std::find_if(notches_.begin(), it_end, 
                    [this, n](auto t) { return t.id == n.id; });
                if (it != it_end)
                    *it = n;
                else
                    notches_.emplace_back(n.id, static_cast<T>(n.time * correction));
            }
            std::sort(notches_.begin(), notches_.end(), 
                [](notch_type a, notch_type b) { return a.time < b.time; });
        }

        notch_type get_notch(unsigned short id)
        {
            auto it_end = notches_.end();
            auto it = std::find_if(notches_.begin(), it_end, 
                [this, id](auto t) { return t.id == id; });
            return (it != it_end) ? *it : notch_type();
        }

    private:
        T start_time_;
        std::vector<notch_type> notches_;
        std::vector<notch_type> raw_notches_;
        cit_notches it_notches_;
        cit_notches it_notches_end_;
    };

    template<numeric T>
    struct extra_data
    {
        T real_time;
    };

    template<numeric To, numeric Tp>
    struct wmg_point
    {
        To omega;
        Tp power;
    };

    using namespace buffers;

    // wmg_buffer_size must be greater than number of the points in 2 periods
    // bdc - bottom dead center
    export
    template<numeric Tt, numeric Tp, size_t wmg_buffer_size, size_t extr_buffer_size>
    class period_analizer final
    {
        using extra_type = extra_data<Tt>;
        using point_type = point<Tt, Tp, extra_type>;
        using period_detector_type = period_detector<Tt, Tp, extra_type, extr_buffer_size>;
        using extr_detector_type = extremum_detector<Tt, Tp, extra_type>;
        using correlation_type = correlation_by_extremums<Tt, Tp, extra_type>;

        constexpr static double precision_x_def = .1;               // relative to the period
        constexpr static double precision_y_def = .1;               // relative to the amplitude
        constexpr static double correlation_criterium_def = .75;    // proportion of correlated extremums
        constexpr static double base_freq_def = 50;
        constexpr static int min_stable_periods = 1;
        constexpr static double one_cycle = 360;                    // degrees
        constexpr static unsigned short bdc_id = 0xFFF0;
        constexpr static int undefined = -1;

    public:
        using omega_type = double;
        using wmg_point_type = wmg_point<omega_type, Tp>;
        using wmg_buffer_type = ring_buffer<wmg_point_type, wmg_buffer_size>;
        using it_wmg_type = wmg_buffer_type::iterator;

        using callback_found = period_detector_type::callback_found;
        using callback_lost = period_detector_type::callback_lost;
        using callback_bdc = std::function<void(Tt x)>;
        using callback_notch = std::function<void(unsigned short id, Tt x)>;
        using callback_wmg = std::function<void(it_wmg_type start, it_wmg_type end)>;

    public:
        period_analizer(
            Tt noise_period,
            Tt period_min,
            Tt period_max,
            callback_bdc cb_bdc,
            callback_notch cb_notch,
            callback_wmg cb_wmg,
            //callback_found cb_found,
            //callback_lost cb_lost
            double prec_x = precision_x_def,
            double prec_y = precision_y_def,
            double correlation_criterium = correlation_criterium_def,
            double base_freq = base_freq_def) noexcept
            : correlation_{}
            , period_detector_(period_min, period_max, prec_x, prec_y,
                correlation_criterium, correlation_,
                [this](point_type& start, point_type& end, double corr, int stab) { 
                    cb_period_found(start, end, corr, stab); 
                }, 
                [this]() { 
                    cb_period_lost(); 
                })
            , extr_detector_(noise_period, 
                [this](point_type& pt) { 
                    cb_extremum_found(pt); 
                })
            , base_freq_{ base_freq }
            , callback_bdc_{ cb_bdc }
            , callback_notch_{ cb_notch }
            , callback_wmg_{ cb_wmg }
            //, callback_found_{ cb_found }
            //, callback_lost_{ cb_lost }
        { }

        bool push(Tt time, Tp power, double frequency) 
        {
            norm_time_ += static_cast<omega_type>((time - time_last_) * frequency / base_freq_);
            time_last_ = static_cast<omega_type>(time);
            wmg_buffer_.push(wmg_point_type(norm_time_, power));
            extr_detector_.push(point_type(static_cast<Tt>(norm_time_), power, { time }));
            check_notches_triggers();
            return period_found_ ? (period_found_ = false, true) : false;
        }

        void check_notches_triggers()
        {
            if (!is_period_stable())
                return;
            if (auto n = notches_.check_next(norm_time_); n.is_valid())
                if (n.id == bdc_id)
                    callback_bdc_(static_cast<Tt>(n.time));
                else
                    callback_notch_(n.id, static_cast<Tt>(n.time));
        }

        bool is_period_stable() const noexcept 
        {
            return stability_ >= min_stable_periods;
        }

        bool set_bdc()
        {
            if (!set_notch(bdc_id))
                return false;
            save_sample_ = true;
            return true;
        }

        bool set_notch(unsigned short id)
        {
            if (!is_period_stable())
                return false;
            return notches_.add_new(id, norm_time_);
        }

    private:
        void cb_extremum_found(point_type& pt)
        {
            period_detector_.push(pt);
        }

        void cb_period_found(const point_type& start, const point_type& end, double correlation, int stability)
        {
            period_start = start;
            period_end = end;
            auto period = end.x - start.x;
            if (save_sample_) {
                etalon_.save_sample(period);
                save_sample_ = false;
            }
            stability_ = stability;
            period_found_ = true;

            if (etalon_.valid()) {
                auto correction = etalon_.get_period() / static_cast<double>(period);
                notches_.add_new_notches(correction);

                auto bdc_notch = notches_.get_notch(bdc_id);
                if (bdc_notch.is_valid())
                { 
                    auto bdc_norm_time = start.x + bdc_notch.time / correction;
                    auto bdc_index = index_by_time(bdc_norm_time);
                    if (last_bdc_ != undefined) {
                        normalize_period(last_bdc_, bdc_index);
                        callback_wmg_(wmg_buffer_.it(last_bdc_), wmg_buffer_.it(bdc_index));
                    }
                    last_bdc_ = bdc_index;
                }
            }
            notches_.start_period(static_cast<omega_type>(end.x));
            //callback_found_(start, end, correlation);
        }

        void cb_period_lost() noexcept
        {
            period_found_ = false;
            last_bdc_ = undefined;
            notches_.cancel_period();
            //callback_lost_();
        }

        void normalize_period(int start, int end)
        {
            auto start_omega = wmg_buffer_[start].omega;
            double correction = one_cycle / static_cast<double>((wmg_buffer_[end].omega - start_omega));
            for (int i = start; i != end; i = wmg_buffer_.next(i))
                wmg_buffer_[i].omega = correction * (wmg_buffer_[i].omega - start_omega);
        }
        
        int index_by_time(omega_type time) // must be refactored to the binary search
        {
            auto end = wmg_buffer_.tail();
            for (auto i = wmg_buffer_.last(); i != end; i = wmg_buffer_.prev(i))
                if (time > wmg_buffer_[i].omega)
                    return i;
            return undefined;
        }

    private:
#ifdef _DEBUG
    public:                             //!!!!!!!!!!!!!!!
#endif // DEBUG
        point_type period_start;
        point_type period_end;
     
        period_detector_type period_detector_;
        extr_detector_type extr_detector_;
        correlation_type correlation_;

    private:
        omega_type norm_time_{};
        omega_type time_last_{};
        
        double base_freq_;
        int stability_{};
        bool save_sample_{ false };
        bool period_found_{ false };

        int last_bdc_{ undefined };

        elalon<Tt> etalon_;
        wmg_buffer_type wmg_buffer_;
        notches<omega_type> notches_;

        callback_found callback_found_;
        callback_lost callback_lost_;
        callback_bdc callback_bdc_;
        callback_notch callback_notch_;
        callback_wmg callback_wmg_;
    };
}