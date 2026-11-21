// Include boundary of the flight executive core (ExVectrControl/executive): pio test -e native -f test_executive_includes
//
// The core may include only <stdint.h>, <stddef.h>, <math.h> and its own headers (ExVectrControl/executive.hpp,
// ExVectrControl/executive/*.hpp), and must not allocate dynamically. This keeps it free of hardware, ExVectrCore,
// the scheduler and the vehicle (docs/project/flight-executive.md, "Include boundary"). The test greps every header
// and source of the core, so a new file is covered automatically.
// Not yet checked: the single-writer rule for actuator calls (design 3.5) - it applies once the CommandApplier exists
// (E5).
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

namespace {

/// Project root, derived from this file's path (PlatformIO compiles it as test/test_executive_includes/test_main.cpp,
/// relative to the project or absolute).
const char *projectRoot() {
  static char root[512];
  static bool done = false;
  if (!done) {
    done = true;
    snprintf(root, sizeof(root), "%s", __FILE__);
    char *cut = strstr(root, "test/test_executive_includes/");
    if (cut)
      *cut = '\0';
  }
  return root;
}

bool endsWith(const char *s, const char *suffix) {
  const size_t n = strlen(s), m = strlen(suffix);
  return n >= m && strcmp(s + n - m, suffix) == 0;
}

/// The include target of a line, or nullptr if the line is not an #include.
bool includeTarget(const char *line, char *out, size_t outLen) {
  while (*line == ' ' || *line == '\t')
    line++;
  if (*line != '#')
    return false;
  line++;
  while (*line == ' ' || *line == '\t')
    line++;
  if (strncmp(line, "include", 7) != 0)
    return false;
  line += 7;
  while (*line == ' ' || *line == '\t')
    line++;
  size_t i = 0;
  while (*line && *line != '\n' && *line != '\r' && *line != ' ' && i + 1 < outLen)
    out[i++] = *line++;
  out[i] = '\0';
  return true;
}

bool allowedInclude(const char *t) {
  if (strcmp(t, "<stdint.h>") == 0 || strcmp(t, "<stddef.h>") == 0 || strcmp(t, "<math.h>") == 0)
    return true;
  if (strcmp(t, "\"ExVectrControl/executive.hpp\"") == 0)
    return true;
  const char *own = "\"ExVectrControl/executive/";
  return strncmp(t, own, strlen(own)) == 0 && endsWith(t, ".hpp\"") && strchr(t + strlen(own), '/') == nullptr;
}

/// Tokens that mean dynamic allocation (checked outside comments only roughly: whole-line comments are skipped).
bool allocates(const char *line) {
  const char *p = line;
  while (*p == ' ' || *p == '\t')
    p++;
  if (p[0] == '/' && (p[1] == '/' || p[1] == '*'))
    return false;
  if (p[0] == '*')
    return false;
  return strstr(line, "malloc(") || strstr(line, "calloc(") || strstr(line, "realloc(") || strstr(line, " new ") ||
         strstr(line, "=new ") || strstr(line, "delete ") || strstr(line, "delete[]");
}

int checkFile(const char *path, int &includes) {
  FILE *f = fopen(path, "r");
  char msg[700];
  snprintf(msg, sizeof(msg), "cannot open %s", path);
  TEST_ASSERT_NOT_NULL_MESSAGE(f, msg);
  char line[512], target[256];
  int lineNo = 0, bad = 0;
  while (fgets(line, sizeof(line), f)) {
    lineNo++;
    if (includeTarget(line, target, sizeof(target))) {
      includes++;
      if (!allowedInclude(target)) {
        snprintf(msg, sizeof(msg), "%s:%d includes %s (outside the executive core boundary)", path, lineNo, target);
        TEST_MESSAGE(msg);
        bad++;
      }
    }
    if (allocates(line)) {
      snprintf(msg, sizeof(msg), "%s:%d dynamic allocation in the executive core", path, lineNo);
      TEST_MESSAGE(msg);
      bad++;
    }
  }
  fclose(f);
  return bad;
}

int checkDir(const char *rel, const char *suffix, int &files, int &includes) {
  char dirPath[600];
  snprintf(dirPath, sizeof(dirPath), "%s%s", projectRoot(), rel);
  DIR *d = opendir(dirPath);
  char msg[700];
  snprintf(msg, sizeof(msg), "cannot open directory %s", dirPath);
  TEST_ASSERT_NOT_NULL_MESSAGE(d, msg);
  int bad = 0;
  while (struct dirent *e = readdir(d)) {
    if (!endsWith(e->d_name, suffix))
      continue;
    char path[900];
    snprintf(path, sizeof(path), "%s/%s", dirPath, e->d_name);
    bad += checkFile(path, includes);
    files++;
  }
  closedir(d);
  return bad;
}

} // namespace

static void test_core_include_boundary() {
  int files = 0, includes = 0, bad = 0;
  bad += checkDir("lib/ExVectrControl/include/ExVectrControl/executive", ".hpp", files, includes);
  bad += checkDir("lib/ExVectrControl/src/executive", ".cpp", files, includes);
  char umbrella[600];
  snprintf(umbrella, sizeof(umbrella), "%slib/ExVectrControl/include/ExVectrControl/executive.hpp", projectRoot());
  bad += checkFile(umbrella, includes);
  files++;
  TEST_ASSERT_TRUE_MESSAGE(files >= 20, "the core files were not found (wrong project root?)");
  TEST_ASSERT_TRUE(includes > files);
  TEST_ASSERT_EQUAL_INT_MESSAGE(0, bad, "include boundary or allocation violations (see messages above)");
}

static void test_checker_rejects_foreign_includes() {
  TEST_ASSERT_TRUE(allowedInclude("<math.h>"));
  TEST_ASSERT_TRUE(allowedInclude("\"ExVectrControl/executive/plan.hpp\""));
  TEST_ASSERT_FALSE(allowedInclude("<cmath>"));
  TEST_ASSERT_FALSE(allowedInclude("\"ExVectrCore/topic.hpp\""));
  TEST_ASSERT_FALSE(allowedInclude("\"ExVectrMath.hpp\""));
  TEST_ASSERT_FALSE(allowedInclude("\"ExVectrControl/control_attitude_tvc.hpp\""));
  TEST_ASSERT_FALSE(allowedInclude("\"ExVectrControl/executive/../../src/x.hpp\""));
  TEST_ASSERT_FALSE(allowedInclude("\"starship_hardware.h\""));
  TEST_ASSERT_TRUE(allocates("  auto *p = new Foo();"));
  TEST_ASSERT_FALSE(allocates("  // a new phase"));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_core_include_boundary);
  RUN_TEST(test_checker_rejects_foreign_includes);
  return UNITY_END();
}
