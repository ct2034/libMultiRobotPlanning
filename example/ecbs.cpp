#include <fstream>
#include <iostream>
#include <numeric>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#define REBUILT_FOCAL_LIST
#include <libMultiRobotPlanning/csv_reader.h>
#include <libMultiRobotPlanning/ecbs.hpp>
#include "timer.hpp"

const uint64_t MAX_FILE_SIZE{1000000};

using libMultiRobotPlanning::ECBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

struct State {
  State(int time, int v) : time(time), v(v) {}

  bool operator==(const State& s) const { return time == s.time && v == s.v; }

  bool equalExceptTime(const State& s) const { return v == s.v; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ": (" << s.v << ")";
  }

  int time;
  int v;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.v);
    return seed;
  }
};
}  // namespace std

///
struct Action {
  Action(int where) : where(where) {}

  int where;
};

std::ostream& operator<<(std::ostream& os, const Action& a) {
  if (a.where > 0)
    os << "to " << a.where - 1 << " ";
  else {
    os << "wait";
  }
  return os;
}

///

struct Conflict {
  enum Type {
    Vertex,
    Edge,
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;

  int v1;
  int v2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    switch (c.type) {
      case Vertex:
        return os << c.time << ": Vertex(" << c.v1 << ")";
      case Edge:
        return os << c.time << ": Edge(" << c.v1 << "," << c.v2 << ")";
    }
    return os;
  }
};

struct VertexConstraint {
  VertexConstraint(int time, int v) : time(time), v(v) {}
  int time;
  int v;

  bool operator<(const VertexConstraint& other) const {
    return std::tie(time, v) < std::tie(other.time, other.v);
  }

  bool operator==(const VertexConstraint& other) const {
    return std::tie(time, v) == std::tie(other.time, other.v);
  }

  friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
    return os << "VC(" << c.time << "," << c.v << ")";
  }
};

namespace std {
template <>
struct hash<VertexConstraint> {
  size_t operator()(const VertexConstraint& s) const {
    size_t hash = 0;
    boost::hash_combine(hash, s.time);
    boost::hash_combine(hash, s.v);
    return hash;
  }
};
}  // namespace std

struct EdgeConstraint {
  EdgeConstraint(int time, int v1, int v2) : time(time), v1(v1), v2(v2) {}
  int time;
  int v1;
  int v2;

  bool operator<(const EdgeConstraint& other) const {
    return std::tie(time, v1, v2) < std::tie(other.time, other.v1, other.v2);
  }

  bool operator==(const EdgeConstraint& other) const {
    return std::tie(time, v1, v2) == std::tie(other.time, other.v1, other.v2);
  }

  friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
    return os << "EC(" << c.time << "," << c.v1 << "," << c.v2 << ")";
  }
};

namespace std {
template <>
struct hash<EdgeConstraint> {
  size_t operator()(const EdgeConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.v1);
    boost::hash_combine(seed, s.v2);
    return seed;
  }
};
}  // namespace std

struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;
  std::unordered_set<EdgeConstraint> edgeConstraints;

  void add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }

  bool overlap(const Constraints& other) {
    std::vector<VertexConstraint> vertexIntersection;
    std::vector<EdgeConstraint> edgeIntersection;
    std::set_intersection(vertexConstraints.begin(), vertexConstraints.end(),
                          other.vertexConstraints.begin(),
                          other.vertexConstraints.end(),
                          std::back_inserter(vertexIntersection));
    std::set_intersection(edgeConstraints.begin(), edgeConstraints.end(),
                          other.edgeConstraints.begin(),
                          other.edgeConstraints.end(),
                          std::back_inserter(edgeIntersection));
    return !vertexIntersection.empty() || !edgeIntersection.empty();
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto& ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    return os;
  }
};

struct Location {
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

///
class Environment {
 public:
  Environment(const std::vector<Location>& nodePositions,
              const std::vector<std::vector<int>>& adjacecyList,
              std::map<std::pair<int, int>, double>& edgew,
              std::vector<int> goals, int meanEdgew)
      : m_nodePositions(nodePositions),
        m_adjacecyList(adjacecyList),
        m_edgew(edgew),
        m_goals(std::move(goals)),
        m_meanEdgew(meanEdgew),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0) {}

  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    assert(constraints);
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto& vc : constraints->vertexConstraints) {
      if (vc.v == m_goals[m_agentIdx]) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
  }

  int admissibleHeuristic(const State& s) {
    Location l = m_nodePositions[s.v];
    Location g = m_nodePositions[m_goals[m_agentIdx]];
    return distNorm(l, g);
  }

  double static distNorm(const Location& a, const Location& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
  }

  // low-level
  int focalStateHeuristic(
      const State& s, int /*gScore*/,
      const std::vector<PlanResult<State, Action, int>>& solution) {
    int numConflicts = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State state2 = getState(i, solution, s.time);
        if (s.equalExceptTime(state2)) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  // low-level
  int focalTransitionHeuristic(
      const State& s1a, const State& s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
      const std::vector<PlanResult<State, Action, int>>& solution) {
    int numConflicts = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State s2a = getState(i, solution, s1a.time);
        State s2b = getState(i, solution, s1b.time);
        if (s1a.equalExceptTime(s2b) && s1b.equalExceptTime(s2a)) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  // Count all conflicts
  int focalHeuristic(
      const std::vector<PlanResult<State, Action, int>>& solution) {
    int numConflicts = 0;

    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            ++numConflicts;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            ++numConflicts;
          }
        }
      }
    }
    return numConflicts;
  }

  bool isSolution(const State& s) {
    return s.v == m_goals[m_agentIdx] && s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int>>& neighbors) {
    // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
    // for(const auto& vc : constraints.vertexConstraints) {
    //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
    //   std::endl;
    // }
    neighbors.clear();
    // Wait
    {
      State n(s.time + 1, s.v);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action(0), m_meanEdgew));
      }
    }
    // Move
    for (uint i = 0; i < m_adjacecyList[s.v].size(); i++) {
      State n(s.time + 1, m_adjacecyList[s.v][i]);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(Neighbor<State, Action, int>(
            n, Action(i + 1), (int)m_edgew[{s.v, n.v}]));
      }
    }
  }

  bool getFirstConflict(
      const std::vector<PlanResult<State, Action, int>>& solution,
      Conflict& result) {
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.v1 = state1.v;
            // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
            // std::endl;
            return true;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.v1 = state1a.v;
            result.v2 = state1b.v;
            return true;
          }
        }
      }
    }

    return false;
  }

  void createConstraintsFromConflict(
      const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      c1.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.v1));
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      c1.edgeConstraints.emplace(
          EdgeConstraint(conflict.time, conflict.v1, conflict.v2));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.edgeConstraints.emplace(
          EdgeConstraint(conflict.time, conflict.v2, conflict.v1));
      constraints[conflict.agent2] = c2;
    }
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

  void getConstraints(Constraints* cs) {
    cs->add(*m_constraints);
  }

 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, int>>& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State& s) {
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;
    return con.find(VertexConstraint(s.time, s.v)) == con.end();
  }

  bool transitionValid(const State& s1, const State& s2) {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.v, s2.v)) == con.end();
  }

 private:
  const std::vector<Location>& m_nodePositions;
  const std::vector<std::vector<int>>& m_adjacecyList;
  std::map<std::pair<int, int>, double>& m_edgew;
  std::vector<int> m_goals;
  int m_meanEdgew;
  size_t m_agentIdx;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
};

int main(int argc, char* argv[]) {
  std::cout << "main"
            << "\n";
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string adjacencylistfile;
  std::string positionsfile;
  std::string jobsfile;
  std::string outputFile;
  float w;
  bool verbose{false};
  desc.add_options()("help", "produce help message")(
      "adjacencylist,a", po::value<std::string>(&adjacencylistfile)->required(),
      "input file with adjacencylist (csv)")(
      "positionsfile,p", po::value<std::string>(&positionsfile)->required(),
      "input file with positionsfile (csv)")(
      "jobsfile,j", po::value<std::string>(&jobsfile)->required(),
      "input file with jobssfile (csv)")(
      "output,o", po::value<std::string>(&outputFile)->required(),
      "output file (YAML)")(
      "suboptimality,w", po::value<float>(&w)->default_value(1.0),
      "suboptimality bound")("verbose,v", "print more info");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
    if (vm.count("verbose")) {
      verbose = true;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  // node positions
  std::ifstream infilep;
  infilep.open(positionsfile);
  char buffernp[MAX_FILE_SIZE];
  infilep.read(buffernp, sizeof(buffernp));
  buffernp[infilep.tellg()] = '\0';
  std::vector<Row> result = parse(buffernp, strlen(buffernp));

  std::vector<Location> np = std::vector<Location>();
  for (size_t r = 0; r < result.size(); r++) {
    Row& row = result[r];
    np.emplace_back(Location(std::stoi(row[0]), std::stoi(row[1])));
  }
  BOOST_ASSERT(np.size() == result.size());
  std::cout << "np.size() " << np.size() << std::endl;

  // adjacency list
  std::ifstream infile;
  infile.open(adjacencylistfile);
  char buffer[MAX_FILE_SIZE];
  infile.read(buffer, sizeof(buffer));
  buffer[infile.tellg()] = '\0';
  result = parse(buffer, strlen(buffer));

  std::vector<std::vector<int>> al;
  std::map<std::pair<int, int>, double> edgew;  // edge weigths
  al.resize(result.size());
  for (size_t r = 0; r < result.size(); r++) {
    Row& row = result[r];
    al[r] = std::vector<int>();
    for (std::string& s : row) {
      size_t n = std::stoi(s);
      if (*row.begin() == s) {
        // first entry of row is node number
        BOOST_ASSERT(n == r);
      } else {
        al[r].push_back(n);
        double d = Environment::distNorm(np[r], np[n]);
        edgew[{r, n}] = d;
        edgew[{n, r}] = d;
      }
    }
  }
  std::cout << "al.size() " << al.size() << std::endl;

  // jobs
  std::ifstream infilej;
  infilej.open(jobsfile);
  char bufferj[MAX_FILE_SIZE];
  infilej.read(bufferj, sizeof(bufferj));
  bufferj[infilej.tellg()] = '\0';
  result = parse(bufferj, strlen(bufferj));

  std::vector<int> goals;
  std::vector<State> startStates;
  for (size_t r = 0; r < result.size(); r++) {
    Row& row = result[r];
    startStates.emplace_back(State(0, std::stoi(row[0])));
    goals.emplace_back(std::stoi(row[1]));
  }
  BOOST_ASSERT(goals.size() == startStates.size());
  std::cout << "startStates.size() " << startStates.size() << std::endl;
  std::cout << "goals.size() " << goals.size() << std::endl;

  int meanEdgew =
      (int)(std::accumulate(
                edgew.begin(), edgew.end(), 0,
                [](double x,
                   std::map<std::pair<int, int>, double>::value_type& v) {
                  return x + v.second;
                }) /
            edgew.size());

  std::cout << "w: " << w << std::endl;
  Environment mapf(np, al, edgew, goals, meanEdgew);
  ECBS<State, Action, int, Conflict, Constraints, Environment> cbs(mapf, w);
  std::vector<PlanResult<State, Action, int>> solution;

  Timer timer;
  bool success = cbs.search(startStates, solution);
  timer.stop();

  if (success) {
    std::cout << "Planning successful! " << std::endl;
    int cost = 0;
    int makespan = 0;
    for (const auto& s : solution) {
      cost += s.cost;
      makespan = std::max<int>(makespan, s.cost);
    }

    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  runtime: " << timer.elapsedSeconds() << std::endl;
    out << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
    out << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {
      if (verbose) {
        std::cout << "Solution for: " << a << std::endl;
        for (size_t i = 0; i < solution[a].actions.size(); ++i) {
          std::cout << solution[a].states[i].second << ": "
                    << solution[a].states[i].first << "->"
                    << solution[a].actions[i].first
                    << "(cost: " << solution[a].actions[i].second << ")"
                    << std::endl;
        }
        std::cout << solution[a].states.back().second << ": "
                  << solution[a].states.back().first << std::endl;
      }

      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].states) {
        out << "    - x: " << np[state.first.v].x << std::endl
            << "      y: " << np[state.first.v].y << std::endl
            << "      t: " << state.second << std::endl;
      }
    }
    Constraints cs;
    mapf.getConstraints(&cs);
    out << "blocks:" << std::endl;
    if(!cs.edgeConstraints.empty()){
      out << "  edgeConstraints:" << std::endl;
      for(EdgeConstraint ec : cs.edgeConstraints){
        out << "    - t: " << ec.time << std::endl
            << "      v1: " << ec.v1 << std::endl
            << "      v1: " << ec.v2 << std::endl;
      }
    }
    if(!cs.vertexConstraints.empty()){
      out << "  vertexConstraints:" << std::endl;
      for(VertexConstraint vc : cs.vertexConstraints){
        out << "    - t: " << vc.time << std::endl
            << "      v: " << vc.v << std::endl;
      }
    }
    if(cs.edgeConstraints.empty() & cs.vertexConstraints.empty()){
      out << "  0" << std::endl;
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  return 0;
}
