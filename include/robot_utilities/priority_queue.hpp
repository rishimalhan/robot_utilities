// author: pradeepr@usc.edu, akabir@usc.edu

#ifndef priorityQ_H
#define priorityQ_H

#include <queue>
#include <tuple>
#include <cmath>

using namespace std;
namespace priorityQ{

typedef std::tuple<int, double, double> queue_entry;

// Custom comparator for queue_entry
const double TIE_BREAK_TOL = 0.05;
struct queue_entry_comparator
{
    inline bool operator() (const queue_entry &a, const queue_entry &b)
    {
        
//         const bool tie_break_needed = (get<2>(a) == get<2>(b));
        const double cost_diff = (get<2>(a) - get<2>(b));
        const bool tie_break_needed = false; //(std::fabs(cost_diff) < TIE_BREAK_TOL);

        if (true == tie_break_needed)
        {
            return std::get<1>(a) < get<1>(b);
        }
        else
        {
            return get<2>(a) > get<2>(b);  
        }
    }
};

// The class that we are interfacing to
class PriorityQueue
{
    priority_queue<queue_entry, std::vector<queue_entry>, queue_entry_comparator > pq;
public:
    void push(const int node_id, const double gcost, const double total_cost)
    {
        pq.push(queue_entry(node_id,gcost,total_cost));
    };
    double top()
    {
        const queue_entry top_entry = pq.top();
        return get<0>(top_entry); // return node_id
    };
    double top_value()
    {
        const queue_entry top_entry = pq.top();
        return get<2>(top_entry); // return node_id
    };
    void pop()
    {
        pq.pop();
    };
    double size()
    {
        return pq.size();
    }
private:
};

}
#endif        