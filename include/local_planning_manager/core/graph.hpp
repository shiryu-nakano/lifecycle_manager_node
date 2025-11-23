#pragma once
#include <vector>
#include <string>
#include <functional>

#include "local_planning_manager/common_types.hpp"

namespace local_planning_manager
{
    class Graph
    {
    public:
        Graph();
        explicit Graph(std::unordered_map<std::string, SemanticState> state_dictionary);

        std::optional<std::string> getCurrentSemanticState(SemanticState semantic) const noexcept;
        /*
        値一致によって現在ノードを特定する。見つからなければ nullptr。
        一致したSemanticStateのidを返すことにする．
        */
        //void ini

    private:
        //内部に持つのはstate_idとSemanticStateの辞書
        std::unordered_map<std::string, SemanticState> state_dictionary_;
    };

} // namespace local_planning_manager