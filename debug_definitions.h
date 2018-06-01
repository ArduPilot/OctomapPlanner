#define RED "\x1B[31m"
#define GRN "\x1B[32m"
#define YEL "\x1B[33m"
#define BLUE "\x1B[34m"
#define RESET "\x1B[0m"

#define Info(str)                                                           \
  do {                                                                       \
    std::cout << __FUNCTION__ << ": Line " << __LINE__ << ": " << GRN << str \
              << RESET << std::endl;                                         \
  } while (false)
#define Warn(str)                                                            \
  do {                                                                       \
    std::cout << __FUNCTION__ << ": Line " << __LINE__ << ": " << YEL << str \
              << RESET << std::endl;                                         \
  } while (false)
#define Error(str)                                                            \
  do {                                                                       \
    std::cout << __FUNCTION__ << ": Line " << __LINE__ << ": " << RED << str \
              << RESET << std::endl;                                         \
  } while (false)
#define Dbg(str)                                                           \
  do {                                                                       \
    std::cout << __FUNCTION__ << ": Line " << __LINE__ << ": " << BLUE << str \
              << RESET << std::endl;                                         \
  } while (false)