#include "local_planning_manager/core/graph.hpp"

namespace local_planning_manager
{
    Graph::Graph() = default;

    // constractor
    Graph::Graph(std::unordered_map<std::string, SemanticState> state_dictionary)
        : state_dictionary_(std::move(state_dictionary))
    {
    }

    std::optional<std::string> Graph::getCurrentSemanticState(SemanticState semantic) const noexcept
    {
        for (const auto& [state_id, state_semantic] : state_dictionary_)
        {
            if (state_semantic == semantic)
            {
                return state_id;
            }
        }
        return std::nullopt;
    }

} // namespace local_planning_manager