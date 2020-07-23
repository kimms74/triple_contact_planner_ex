#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;


template <typename T>
vector<vector<T>> get_combinations_from_vector(vector<T> set, int pick_up){
    //! 요소의 개수가 조합의 크기보다 작다면 조합은 실패한다.
    vector<vector<T>> combinations;
    if(set.size() < pick_up) throw exception();

    //! 현재 가지고있는 요소의 위치를 삽입한다.
    //! vector<T> 에서는 모든 요소의 위치를 삽입한다.
    vector<int> index_mapper;
    for(auto i=0ULL; i<set.size(); i++) index_mapper.push_back(i);

    //! 조합을 위한 이터레이터를 선언한다.
    vector<int> combination_iterator;
    for(auto i=0;i<pick_up;i++) combination_iterator.push_back(1);
    for(auto i=0ULL;i<set.size()-pick_up;i++) combination_iterator.push_back(0);
    sort(combination_iterator.begin(), combination_iterator.end());

    //! 매 루프마다 조합을 조립한다.
    do {
        vector<T> this_combination;
        for(auto i=0ULL; i<combination_iterator.size(); i++){
            if(combination_iterator[i]) {
                this_combination.push_back(set[index_mapper[i]]);
            }
        }
        combinations.push_back(this_combination);
    } while(next_permutation(combination_iterator.begin(), combination_iterator.end()));

    //! 완성된 조합의 배열을 반환한다.
    return combinations;
};

// double Arr[9];
// double Select[9];
template <typename T>
void DFS(vector<vector<T>> &combinations, vector<T> &set)
{    
    double step_size = 0.5;
    if (set.size() == 3)
    {
        combinations.push_back(set);
        return;
    }
 
    for (int i = 0; i * step_size <= 1; ++i)
    {
        set.emplace_back(i * step_size);
        DFS(combinations, set);
        set.pop_back();
    }
}
