from collections import deque

from march_shared_classes.exceptions.gait_exceptions import SubgaitGraphError


class SubgaitGraph(object):
    START = 'start'
    END = 'end'
    TO = 'to'
    STOP = 'stop'
    TRANSITIONS = [TO, STOP]

    def __init__(self, graph):
        self._graph = graph
        self.validate()

    def validate(self):
        """Validates the graph and raises an exception when not valid.

        This method checks a few things to prove consistency:
        1. Checks that a `start` and `end` state exist
        2. Checks that is possible to get to every state from `start`
        3. Checks that it is possible to get from every state to `end`
        4. Checks that all subgaits do not have equal `stop` and `to` transitions
        """
        if self.START not in self._graph:
            raise SubgaitGraphError('There is no state {s}'.format(s=self.START))

        # Do a breadth-first search to check for validity
        queue = deque([(self.START, set())])
        visited = {}
        while len(queue) != 0:
            name, from_subgaits = queue.popleft()
            if name in visited:
                visited[name] = visited[name].union(from_subgaits)
                continue

            visited[name] = from_subgaits
            if name == self.END:
                continue

            self._validate_subgait(name)
            subgait = self._graph[name]
            for transition in self.TRANSITIONS:
                if transition in subgait:
                    from_subgaits = from_subgaits.copy()
                    from_subgaits.add(name)
                    queue.append((subgait[transition], from_subgaits))

        self._validate_visited(visited)

    def _validate_subgait(self, name):
        subgait = self._graph.get(name)
        if subgait is None:
            raise SubgaitGraphError('Subgait {n} is not a subgait in the graph'.format(n=name))
        if not any([transition in subgait for transition in self.TRANSITIONS]):
            raise SubgaitGraphError('Subgait {n} has no transitions'.format(n=name))
        if not all([transition in self.TRANSITIONS for transition in subgait]):
            raise SubgaitGraphError(
                'Subgait {n} has unknown transitions. Available transitions {t}'.format(n=name, t=self.TRANSITIONS))
        if len(set(subgait.values())) != len(subgait):
            raise SubgaitGraphError('Subgait {n} transitions cannot be equal'.format(n=name))

    def _validate_visited(self, visited):
        if len(visited[self.START]) != 0:
            raise SubgaitGraphError('Transition to `{s}` is not allowed'.format(s=self.START))
        if self.END not in visited:
            raise SubgaitGraphError('There are no transitions to `{e}`'.format(e=self.END))
        if len(visited[self.END]) != len(self._graph):
            not_covered = set(self._graph).difference(visited[self.END])
            raise SubgaitGraphError('`{e}` is not reachable from {s}'.format(e=self.END, s=not_covered))

    def __contains__(self, subgait_name):
        """Checks if the given subgait name is contained in the graph."""
        return subgait_name in self._graph

    def __getitem__(self, transition):
        """Returns the subgait the given subgait transitions to.

        :param (str, str) transition: Tuple of subgait name and type of transition, can be either 'to' or 'stop'
        :rtype str
        :returns Name of subgait that the given transition transitions to
        """
        subgait_name, transition_type = transition
        if subgait_name not in self._graph:
            raise KeyError('Gait does not contain subgait {0}'.format(subgait_name))
        return self._graph[subgait_name].get(transition_type)

    def __iter__(self):
        """Returns an iterator over all possible transitions in arbitrary order.

        Excludes 'start' and 'end'.
        """
        return iter([(from_subgait, to_subgait)
                     for from_subgait, transitions in self._graph.items()
                     for to_subgait in transitions.values()
                     if len({from_subgait, to_subgait} & {self.START, self.END}) == 0])

    def __eq__(self, other):
        return isinstance(other, SubgaitGraph) and self._graph == other._graph

    def __ne__(self, other):
        return not self == other