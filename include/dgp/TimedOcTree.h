/**
 * TODO: write snippet to explain what this file is for
 * REF: OcTree.h
 */

#ifndef TIMED_OCTOMAP_OCTREE_H
#define TIMED_OCTOMAP_OCTREE_H

#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/OcTreeNode.h>
#include <octomap/ScanGraph.h>

#include "TimedOcTreeNode.h"

namespace octomap {

    class TimedOcTree : public octomap::OccupancyOcTreeBase<TimedOcTreeNode> 
    {
        public:
            TimedOcTree(double resolution, double decay_duration) : octomap::OccupancyOcTreeBase<TimedOcTreeNode>(resolution) {decay_duration_ = decay_duration;}

            // Create a new instance of this tree
            TimedOcTree* create() const override {
                return new TimedOcTree(this->getResolution(), decay_duration_);
            }

            // Return a string identifying the tree type
            std::string getTreeType() const override {
                return "TimedOcTree";
            }

            void updateNodeTimestamp(const octomap::OcTreeKey& key, double timestamp) {
                auto* node = this->search(key);
                if (node) {
                    node->updateTimestamp(timestamp);
                }
            }

            void decayNodes(double current_time) 
            {
                
                // updateNode() is likely to change the structure of the tree, so we need to collect the nodes first and update them afterwards
                // If you don't collect the nodes first, you will get sgmentation fault
                std::vector<octomap::OcTreeKey> nodes_to_update;
                for (auto it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it) 
                {
                    if (current_time - it->getTimestamp() > decay_duration_) 
                    {
                        nodes_to_update.push_back(it.getKey());
                    }
                }

                // Update nodes outside of the iteration
                for (const auto& key : nodes_to_update) 
                {
                    this->updateNode(key, false); // Mark as free
                }

            }

            double getDecayDuration() const {
                return decay_duration_;
            }


        protected:

            double decay_duration_;
    };

} // end namespace

#endif
