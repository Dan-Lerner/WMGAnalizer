module;
#include<math.h>
#include<concepts>
#include<algorithm>
#include<iterator>
#include<functional>
export module periodanalizer;

import buffers;

namespace trend_analizer
{
    using namespace buffers;

    export
    template <typename T>
    concept numeric = std::integral<T> || std::floating_point<T>;

    export
    template<numeric Tx, numeric Ty, typename Tex>
    struct point final
    {
        Tx x;
        Ty y;
        Tex extra;
    };

    export
    template<numeric Tx, numeric Ty, typename Tex>
    class correlation_by_extremums final
    {
        using point_type = point<Tx, Ty, Tex>;

    public:
        correlation_by_extremums() noexcept
            : points_count_{}
            , points_correlate_{}
        {}

        bool is_same_x(Tx x1, Tx x2) 
        {
            return fabs(static_cast<double>(x1 - x2)) < precision_x_;
        }
        
        bool is_same_y(Ty y1, Ty y2) 
        {
            return fabs(static_cast<double>(y1 - y2)) < precision_y_;
        }
        
        bool is_same_point(const point_type& pt1, const point_type& pt2) 
        {
            return is_same_x(pt1.x, pt2.x) && is_same_y(pt1.y, pt2.y);
        }
        
        void start_new_set(Tx difference) noexcept
        {
            difference_ = difference;
            points_count_ = 0;
            points_correlate_ = 0;
        }
        
        template<class It>
        double push(const point_type& pt, It& it_etalon) 
        {
            while (true) {                                      //// !!!!!!!!! add it_etalon_end
                auto&& pt_et = *it_etalon;
                auto&& x = pt.x - difference_;
                if (is_same_x(pt_et.x, x)) {
                    if (is_same_y(pt_et.y, pt.y))
                        points_correlate_ += 2;
                    points_count_ += 2;
                    ++it_etalon;
                    break;
                }
                if (x < pt_et.x) {
                    ++points_count_;
                    break;
                }
                ++it_etalon;
                ++points_count_;
            }
            return calc_criterium();
        }

        template<class It, class It_etalon>
        double check_correllation(It&& it_begin, It&& it_end, It_etalon&& it_etalon, Tx difference)
        {
            start_new_set(difference);
            for (auto&& i = it_begin; i != it_end; ++i)
                push(*i, it_etalon);
            return calc_criterium();
        }

        void set_precision_x(Tx precision) noexcept { precision_x_ = precision; }
        void set_precision_y(Ty precision) noexcept { precision_y_ = precision; }

    private:
        double calc_criterium() 
        {
            return points_correlate_ / static_cast<double>(points_count_);
        }

    public:
        Tx precision_x_;
        Ty precision_y_;
        int points_count_;
        int points_correlate_;
        Tx difference_;
    };

    export
    template<numeric Tx, numeric Ty, typename Tex, int buffer_size>
    class period_detector final
    {
        using point_type = point<Tx, Ty, Tex>;
        using buffer_type = ring_buffer<point_type, buffer_size>;
        using it_type = buffer_type::iterator;
        using r_it_type = buffer_type::reverse_iterator;
        using correlation_type = correlation_by_extremums<Tx, Ty, Tex>;

        constexpr static int undefined = -1;
        constexpr static int min_points_to_detect = 5;

    public:
        using callback_found = std::function<void(point_type& start, point_type& end, 
            double correlation, int stability)>;
        using callback_lost = std::function<void()>;

    public:
        period_detector(
            Tx period_min, 
            Tx period_max,
            double prec_x,
            double prec_y,
            double correlation_criterium,
            correlation_type& correlation_strategy,
            callback_found cb_found,
            callback_lost cb_lost)
            : prec_x_criterium_{ prec_x }
            , prec_y_criterium_{ prec_y }
            , period_max_{ period_max }
            , period_min_{ period_min }
            , corr_criterium_{ correlation_criterium }
            , callback_found_{ cb_found }
            , callback_lost_{ cb_lost }
            , corr_strategy_{ correlation_strategy }
            , min_last_{ undefined }
            , max_last_{ undefined }
            , periods_count_{ 0 }
        { }

        bool push(const point_type& pt) 
        {
            if (buffer_.count() > 0 && corr_strategy_.is_same_y(pt.y, buffer_[buffer_.last()].y))
                return false;
            auto index = buffer_.push(pt);
            return periods_count_ > 0 ? process_new_period(index) : find_period(index);
        }

        int get_stable_periods()
        {
            return periods_count_;
        }

    private:
        bool process_new_period(int index)
        {
            point_type pt{ buffer_[index] };
            auto correlation = corr_strategy_.push(pt, it_etalon_);
            // the correction of the period tension must be inserted
            pt.x -= period_;
            if (corr_strategy_.is_same_point(buffer_[last_period_end_], pt)) {
                if (correlation > corr_criterium_) {
                    period_found(last_period_end_, index, correlation);
                    return true;
                }
            }
            if (pt.x > buffer_[last_period_end_].x)
                period_lost();
            return false;
        }

        bool find_period(int index)
        {
            if (buffer_.overheaded()) {
                auto tail_index = buffer_.tail();
                if (tail_index == min_last_ || tail_index == max_last_)
                    update_min_max(tail_index);
            }
            auto min_prev = min_last_;
            if (check_candidate(index))
                return check_correlation(index, min_prev);
            return false;
        }

        void period_found(int start, int end, double correlation)
        {
            ++periods_count_;
            last_period_end_ = end;
            it_etalon_ = ++buffer_.it(start);
            period_ = buffer_[end].x - buffer_[start].x;
            corr_strategy_.start_new_set(period_);
            callback_found_(buffer_[start], buffer_[end], correlation, periods_count_);
        }

        void period_lost()
        {
            periods_count_ = 0;
            min_last_ = undefined;
            max_last_ = undefined;
            period_ = 0;
            corr_strategy_.set_precision_x(0);
            corr_strategy_.set_precision_y(0);
            callback_lost_();
        }

        bool check_candidate(int index) 
        {
            if (min_last_ < 0) {                 // 1st time
                set_min_max(index, index);
                return false;
            }
            if (check_as_min(index))
                return true;
            if (check_as_max(index))
                return /*true*/ false;
            return false;
        }
        
        bool check_as_max(int index) 
        {
            if (corr_strategy_.is_same_y(buffer_[index].y, buffer_[max_last_].y)) {
                set_min_max(min_last_, index);
                return true;
            }
            if (buffer_[index].y > buffer_[max_last_].y)
                set_min_max(min_last_, index);
            return false;
        }

        bool check_as_min(int index) 
        {
            if (corr_strategy_.is_same_y(buffer_[index].y, buffer_[min_last_].y)) {
                set_min_max(index, max_last_);
                return true;
            }
            if (buffer_[index].y < buffer_[min_last_].y)
                set_min_max(index, max_last_);
            return false;
        }

        void set_min_max(int min, int max) {
            min_last_ = min;
            max_last_ = max;
            corr_strategy_.set_precision_y(
                static_cast<double>(buffer_[max_last_].y - buffer_[min_last_].y) * prec_y_criterium_);
        }

        bool check_correlation(int extr_new, int extr_last) 
        {
            if (buffer_.count() < min_points_to_detect)
                return false;
            Tx period_to_test;
            while (extr_last != undefined) {
                period_to_test = buffer_[extr_new].x - buffer_[extr_last].x;
                if (period_to_test < period_min_)
                    return false;
                if (period_to_test > period_max_ ||
                    buffer_.difference(extr_new, extr_last) * 2 > buffer_.count() ||
                    buffer_[extr_new].x - buffer_[buffer_.tail()].x < 2 * period_to_test)
                {
                    update_min_max(extr_last);
                    return false;
                }
                corr_strategy_.set_precision_x(static_cast<Tx>(period_to_test * prec_x_criterium_));
                auto correlation = corr_strategy_.check_correllation(buffer_.r_it(extr_new),
                    buffer_.r_it(extr_last), buffer_.r_it(extr_last), period_to_test);
                if (correlation > corr_criterium_) {
                    period_found(extr_last, extr_new, correlation);
                    return true;
                }
                extr_last = find_prev_extr(extr_last);
            }
            return false;
        }

        int find_prev_extr(int index) 
        {
            auto extr = buffer_[index].y;
            auto end = buffer_.tail();
            index = buffer_.prev(index);
            while (index != end) {
                if (corr_strategy_.is_same_y(buffer_[index].y, extr))
                    return index;
                index = buffer_.prev(index);
            }
            return undefined;
        }

        void update_min_max(int index) 
        {
            index = buffer_.next(index);
            set_min_max(index, index);
            auto end{ buffer_.head() };
            while (index != end) {
                check_as_min(index);
                check_as_max(index);
                index = buffer_.next(index);
            }
        }

    public:
        buffer_type::iterator begin() { return buffer_.begin(); }
        buffer_type::iterator end() { return buffer_.end(); }
        buffer_type::const_iterator cbegin() const { return buffer_.cbegin(); }
        buffer_type::const_iterator cend() const { return buffer_.cend(); }
        buffer_type::reverse_iterator rbegin() { return buffer_.rbegin(); }
        buffer_type::reverse_iterator rend() { return buffer_.rend(); }

    private:
        int min_last_;
        int max_last_;
        int last_period_end_;

        Tx period_max_;
        Tx period_min_;

        double corr_criterium_;
        double prec_x_criterium_;
        double prec_y_criterium_;

        buffer_type buffer_;
        correlation_type& corr_strategy_;
        
        int periods_count_;
        Tx period_;
        it_type it_etalon_;
        callback_found callback_found_;
        callback_lost callback_lost_;
    };

    export
    template<numeric Tx, numeric Ty, typename Tex>
    class extremum_detector final
    {
        using point_type = point<Tx, Ty, Tex>;
        using callback = std::function<void(point_type& pt)>;

        struct tracker
        {
            void init(const point_type& pt) noexcept 
            {
                last_extremum = pt;
                last_trend = pt;
                trend_found = false;
            }

            point_type last_extremum{};
            point_type last_trend{};
            bool trend_found{};
        };

    public:
        extremum_detector(Tx noise, callback cb) noexcept
            : noise_period_{ noise }
            , callback_{ cb }
        {}

        bool push(const point_type& pt) 
        {
            if (pt.y < track_min_.last_trend.y)
                return is_extremum(track_min_, pt, track_max_);
            if (pt.y > track_max_.last_trend.y)
                return is_extremum(track_max_, pt, track_min_);
            return false;
        }

    private:
        bool is_extremum(tracker& track, const point_type& pt, tracker& opp_track) 
        {
            track.last_trend = pt;
            if (fabs(static_cast<double>(track.last_trend.x - track.last_extremum.x)) > noise_period_) {
                if (!track.trend_found) {
                    track.trend_found = true;
                    if (opp_track.trend_found) {
                        callback_(track.last_extremum);
                        return true;
                    }
                }
                opp_track.init(pt);
            }
            return false;
        }

    private:
        callback callback_;
        Tx noise_period_;

#ifdef _DEBUG
    public:
#endif
        tracker track_min_;
        tracker track_max_;
    };
}