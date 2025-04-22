/**
 * TODO: write snippet to explain what this file is for
 * REF: OcTreeNode.h
 */

#ifndef TIMED_OCTOMAP_OCTREE_NODE_H
#define TIMED_OCTOMAP_OCTREE_NODE_H

#include <octomap/octomap_types.h>
#include <octomap/octomap_utils.h>
#include <octomap/OcTreeDataNode.h>
#include <limits>

namespace octomap {

    class TimedOcTreeNode : public OcTreeNode 
    {
    public:
        TimedOcTreeNode() : last_update_time(0) {}

        virtual ~TimedOcTreeNode() = default; // Polymorphic type now

        void updateTimestamp(double timestamp) {
            last_update_time = timestamp;
        }

        double getTimestamp() const {
            return last_update_time;
        }

    protected:
        double last_update_time;
    };


} // end namespace

#endif
