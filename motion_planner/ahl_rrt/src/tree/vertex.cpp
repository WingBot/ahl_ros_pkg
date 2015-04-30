#include "ahl_rrt/tree/vertex.hpp"

using namespace ahl_rrt;

void Vertex::addChild(VertexPtr& child)
{
  child_.push_back(child);
  child->setParent(shared_from_this());
}

void Vertex::setParent(const VertexPtr& parent)
{
  parent_ = parent;
}
