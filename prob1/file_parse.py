def read_line(line):
    contents = []
    words = [word.strip() for word in line.split(' ')]
    for i in range(len(words) // 2):
        contents.append([float(words[2 * i]), float(words[2 * i + 1])])
    return contents


def parse_problem(world_file, problem_file):
    with open(world_file, 'r', encoding='utf-8') as f:
        content_wolrd = f.read().strip().split('\n')
    robot = read_line(content_wolrd[0].strip())
    obstacles = [read_line(line.strip()) for line in content_wolrd[1:]]

    with open(problem_file, 'r', encoding='utf-8') as f:
        content_prob = f.read().strip()
    probs = [{'start': (line[0], line[1]), 'goal': (line[2], line[3])} for line in content_prob.split(' ')]
    return robot, obstacles, probs
