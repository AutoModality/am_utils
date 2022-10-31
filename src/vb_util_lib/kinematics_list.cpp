/*
 * am_sp_kinematics_list.cpp
 *
 *  Created on: Jun 3, 2015
 *      Author: ubuntu
 */

#include <vb_util_lib/kinematics_list.h>

KinematicsList::KinematicsList(rclcpp::Node::SharedPtr node): node_(node)
{
    // TODO Auto-generated constructor stub

}

KinematicsList::~KinematicsList() {
    // TODO Auto-generated destructor stub
}

bool KinematicsList::deleteNode(std::string id) {
    KinematicsNode* p = findNodePtr(id);
    if (p != NULL)
    {
        deleteNode(p);
        return true;
    }

    return false;
}

Kinematics* KinematicsList::findNode(std::string id) {
    KinematicsNode* p = findNodePtr(id);

    if (p != NULL)
    {
        return p->node;
    }

    return  NULL;
}

Kinematics* KinematicsList::updateNode(const brain_box_msgs::msg::TargetPose* pose) {
    KinematicsNode* p = findNodePtr(pose->target_id);
    //	std::cout << "updateNode" << pose->target_id;
    if (p == NULL)
    {
        // new one so add it to existing list
        //		std::cout << " adding\n";
        geometry_msgs::msg::PoseStamped ps;
        ps.header = pose->header;
        ps.pose = pose->pose;
        Kinematics* np = new Kinematics(node_, ps, pose->target_id);
        addNode(np);
        return(np);
    }
    else
    {
        // Already exists so update attributes
        //		std::cout << " updating " << p->id << "\n";;
        p->node->setPose(pose->header, pose->pose);
        return(p->node);
    }
}

void KinematicsList::addNode(Kinematics* p) {
    KinematicsNode* node = new KinematicsNode(node_, p);
    if (head != NULL)
    {
        // Push the node to the end of the list
        node->next = head;
        node->prev = head->prev;
        head->prev->next = node;
        head->prev = node;
    }
    else
    {
        head = node;
        node->next = node;
        node->prev = node;
    }

    cnt++;
}

KinematicsList::KinematicsNode::KinematicsNode(rclcpp::Node::SharedPtr n, Kinematics* p): nh_(n)
{
    node = p;
    if (p != NULL) {
        id = p->getObjectId();
    }
}

KinematicsList::KinematicsNode::~KinematicsNode() {
    if (node != NULL)
    {
        delete node;
    }
}

KinematicsList::KinematicsNode* KinematicsList::findNodePtr(std::string id) {
    if (head == NULL)
    {
        return NULL;
    }

    KinematicsNode* p = head;
    for (int i = 0; i < cnt; i++)
    {
        if (p->id == id)
        {
            return p;
        }
        p = p->next;
    }

    return NULL;
}

Kinematics* KinematicsList::trackNode(std::string id) {
    KinematicsNode* p = findNodePtr(id);
    if (p != NULL)
    {
        focus_node = p;
        return p->node;
    }

    return NULL;
}

/** Updates the list of targets being tracked from the list of targets reported in the ROS topic.
 *
 * Updates existing targets with new attribute information and adds new targets that are
 * not already in the list.
 * Returns the focus node iif it was in the list of targets reported. If not in the reported
 * target list then it returns NULL.
 * Note that if there is not an existing focus node then the first node in the list becomes the
 * default focus node.
 */
Kinematics* KinematicsList::updateList(
        const brain_box_msgs::msg::TargetPoseArray::SharedPtr msg) {

    Kinematics* f_node = NULL;
    for (int i = 0; i < msg->poses.size(); i++)
    {
        const brain_box_msgs::msg::TargetPose* p = &(msg->poses.at(i));
        //		std::cout << "target(" << i << ") " << p->target_id << "\n";
        const Kinematics* n = updateNode(p);

        // Check to see if this is the focus node
        if (isFocus(n))
        {
            f_node = focus_node->node;
        }
    }

    // Set the default focus node to the head if there is not already one
    if ((cnt > 0) && (focus_node == NULL))
    {
        focus_node = head;
        f_node = focus_node->node;
    }

    return f_node;
}

void KinematicsList::print(std::string s) {
    std::cout << s << " ";
    print();
}

void KinematicsList::print() {
    std::cout << "LIST ";
    if (head != NULL)
    {
        std::cout << "(" << cnt << ")  ";
        if (cnt > 0)
        {
            std::cout << "[" << focus_node->id << "]  ";
        }
        KinematicsNode* n = head;
        for (int i = 0; i < cnt; i++)
        {
        	geometry_msgs::msg::Pose p;
        	n->node->getPose(p);
            std::cout << "    " << n->id << "," << p.position.x << "," << p.position.y << "," << p.position.z;
            n = n->next;
        }
        std::cout << "\n";
    }
    else
    {
        std::cout << "EMPTY\n";
    }
}

void KinematicsList::deleteNode(KinematicsNode* p) {
    if (cnt > 1)
    {
        p->next->prev = p->prev;
        p->prev->next = p->next;
        if (head == p)
        {
            head = p->next;
        }
        if (focus_node == p)
        {
            focus_node = NULL;
        }
    }
    else
    {
        head = NULL;
    }
    delete p;
    cnt--;
}

