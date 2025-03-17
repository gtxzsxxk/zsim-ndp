#ifndef INTERVAL_RECORDER_H_
#define INTERVAL_RECORDER_H_

#include <cstdint>
#include "galloc.h"
#include "intrusive_list.h"

/**
 * An interval recorder that stores non-overlapped intervals.
 *
 * Support adding new intervals and merging with current ones.
 */

class IntervalRecorder {
    public:
        IntervalRecorder() : intervals(), origin(0) {}

        void addInterval(uint64_t begin, uint64_t end) {
            if (begin >= end) return;
            if (end <= origin) return;
            if (begin < origin) begin = origin;

            // During adding the new interval, we find all the existing intervals that overlap with
            // the new one, and merge them into the new one. After finding and merging all, we insert
            // the new interval.
            auto p = intervals.front();
            while (p != nullptr) {
                if (end < p->begin) {
                    // No overlap with following ones, insert the new interval.
                    auto newInterval = new Interval(begin, end);
                    if (p->prev == nullptr) intervals.push_front(newInterval);
                    else intervals.insertAfter(p->prev, newInterval);
                    return;
                } else if (begin > p->end) {
                    // No overlap but before the current one, do nothing.
                    p = p->next;
                } else {
                    // Overlaps with current one, remove it and merge into the new one.
                    begin = std::min(begin, p->begin);
                    end = std::max(end, p->end);
                    auto r = p;
                    p = p->next;
                    intervals.remove(r);
                    delete r;
                }
            }
            // Push to the end.
            auto newInterval = new Interval(begin, end);
            intervals.push_back(newInterval);
        }

        void updateOrigin(uint64_t _origin) {
            origin = _origin;

            // Remove all intervals before the new origin.
            auto p = intervals.front();
            while (p != nullptr) {
                if (p->end <= origin) {
                    auto q = p;
                    p = p->next;
                    intervals.remove(q);
                    delete q;
                } else if (p->begin < origin) {
                    p->begin = origin;
                    break;
                } else {
                    break;
                }
            }
            assert(intervals.empty() || intervals.front()->begin < intervals.front()->end);
            assert(intervals.empty() || intervals.front()->begin >= origin);
        }

        uint64_t getCoverage(uint64_t end) const {
            uint64_t coverage = 0;
            for (auto p = intervals.front(); p != nullptr; p = p->next) {
                if (end <= p->begin) {
                    break;
                } else if (end <= p->end) {
                    coverage += end - p->begin;
                    break;
                } else {
                    coverage += p->end - p->begin;
                }
            }
            return coverage;
        }

    private:
        struct Interval : InListNode<Interval>, GlobAlloc {
            // Interval is [begin, end)
            uint64_t begin;
            uint64_t end;

            Interval(uint64_t _begin, uint64_t _end) : begin(_begin), end(_end) {}

            // Use glob mem
            using GlobAlloc::operator new;
            using GlobAlloc::operator delete;
        };

        InList<Interval> intervals;

        // The smallest number for all intervals. Intervals smaller than it will be ignored.
        uint64_t origin;
};

#endif  // INTERVAL_RECORDER_H_

