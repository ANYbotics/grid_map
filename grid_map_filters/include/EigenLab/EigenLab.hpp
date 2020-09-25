// --*-  Mode: C++; c-basic-offset:4; indent-tabs-mode:t; tab-width:4 -*--
// EigenLab
// Version: 1.0.0
// Author: Dr. Marcel Paz Goldschen-Ohm
// Email:  marcel.goldschen@gmail.com
// Copyright (c) 2015 by Dr. Marcel Paz Goldschen-Ohm.
// Licence: MIT
//----------------------------------------

#ifndef EIGENLAB__EIGENLAB_HPP_
#define EIGENLAB__EIGENLAB_HPP_

#include <Eigen/Dense>
#include <iomanip>
#include <type_traits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// Define both DEBUG and EIGENLAB_DEBUG for step-by-step equation parsing printouts.
#ifndef DEBUG
// #define DEBUG
#endif

#ifndef EIGENLAB_DEBUG
// #define EIGENLAB_DEBUG
#endif

#ifdef DEBUG
# include <iostream>
#endif

namespace EigenLab
{
//----------------------------------------
// A wrapper for a matrix whose data is either stored locally or shared.
//
// Template typename Derived can be any dynamically sized matrix type supported by Eigen.
//
// !!! matrix() promises to ALWAYS return a map to the matrix data whether it's
// stored locally or externally in some shared memory.
//
// !!! local() provides direct access to the local data, but this data is
// ONLY valid when isLocal() is true. In most cases, you're best off
// accessing the matrix data via matrix() instead.
//----------------------------------------
template<typename Derived = Eigen::MatrixXd>
class Value
{
private:
  // Local matrix data.
  Derived mLocal;

  // Map to shared matrix data (map points to mLocal if the data is local).
  // !!! This map promises to ALWAYS point to the matrix data whether it's
  // stored locally in mLocal or externally in some shared memory.
  Eigen::Map<Derived> mShared;

  // Flag indicating whether the local data is being used.
  bool mIsLocal;

public:
  // Access mapped data (whether its local or shared).
  // !!! matrix() promises to ALWAYS return a map to the matrix data whether it's
  // stored locally in mLocal or externally in some shared memory.
  inline Eigen::Map<Derived> & matrix()
  {
    return mShared;
  }
  inline const Eigen::Map<Derived> & matrix() const {return mShared;}

  // Access local data.
  // !!! WARNING! This data is ONLY valid if isLocal() is true.
  // !!! WARNING! If you change the local data via this method,
  // you MUST call mapLocal() immediately afterwards.
  // In most cases, you're best off accessing the matrix data via matrix() instead.
  inline Derived & local() {return mLocal;}
  inline const Derived & local() const {return mLocal;}

  // Is mapped data local?
  inline bool isLocal() const {return mIsLocal;}

  // Set mapped data to point to local data.
  inline void mapLocal()
  {
    new(&mShared) Eigen::Map<Derived>(mLocal.data(), mLocal.rows(), mLocal.cols());
    mIsLocal = true;
  }

  //  Copy shared data to local data (if needed).
  inline void copySharedToLocal() {if (!isLocal()) {mLocal = mShared; mapLocal();}}

  // Set local data.
  Value()
  : mLocal(1, 1), mShared(mLocal.data(), mLocal.rows(), mLocal.cols()), mIsLocal(true)
  {
  }
  explicit Value(const typename Derived::Scalar s)
  : mLocal(Derived::Constant(1, 1, s)), mShared(
      mLocal.data(), mLocal.rows(), mLocal.cols()), mIsLocal(true)
  {
  }
  explicit Value(const Derived & mat)
  : mLocal(mat), mShared(mLocal.data(), mLocal.rows(), mLocal.cols()),
    mIsLocal(true)
  {
  }
  inline void setLocal(const typename Derived::Scalar s)
  {
    mLocal = Derived::Constant(1, 1, s); mapLocal();
  }
  inline void setLocal(const Eigen::MatrixBase<Derived> & mat) {mLocal = mat; mapLocal();}
  inline void setLocal(const Value & val) {mLocal = val.matrix(); mapLocal();}
  inline void setLocal(const typename Derived::Scalar * data, size_t rows = 1, size_t cols = 1)
  {
    setShared(data, rows, cols); copySharedToLocal();
  }
  inline Value & operator=(const typename Derived::Scalar s) {setLocal(s); return *this;}
  inline Value & operator=(const Derived & mat) {setLocal(mat); return *this;}

  // Set shared data.
  explicit Value(const typename Derived::Scalar * data, size_t rows = 1, size_t cols = 1)
  : mShared(
      const_cast<typename Derived::Scalar *>(data), rows, cols), mIsLocal(false)
  {
  }
  inline void setShared(const typename Derived::Scalar * data, size_t rows = 1, size_t cols = 1)
  {
    new(&mShared) Eigen::Map<Derived>(const_cast<typename Derived::Scalar *>(data), rows, cols);
    mIsLocal = false;
  }
  inline void setShared(const Derived & mat) {setShared(mat.data(), mat.rows(), mat.cols());}
  inline void setShared(const Value & val)
  {
    setShared(val.matrix().data(), val.matrix().rows(), val.matrix().cols());
  }

  // Set to local or shared data dependent on whether val
  // maps its own local data or some other shared data.
  Value(const Value & val)
  : mLocal(1, 1), mShared(mLocal.data(), mLocal.rows(), mLocal.cols())
  {
    (*this) = val;
  }
  inline Value & operator=(const Value & val)
  {
    if (val.isLocal()) {
      setLocal(val);
    } else {
      setShared(val);
    } return *this;
  }
};
typedef Value<Eigen::MatrixXd> ValueXd;
typedef Value<Eigen::MatrixXf> ValueXf;
typedef Value<Eigen::MatrixXi> ValueXi;

// check if a class has a comparison operator (ie. std::complex does not)
template<typename T>
struct has_operator_lt_impl
{
  template<class U>
  // deepcode ignore CopyPasteError: We will not change third party code yet.
  static auto test(U *)->decltype(std::declval<U>() < std::declval<U>());
  template<typename>
  static auto test(...)->std::false_type;
  using type = typename std::is_same<bool, decltype(test<T>(0))>::type;
};
template<typename T>
struct has_operator_lt : has_operator_lt_impl<T>::type {};

//----------------------------------------
// Equation parser.
//
// Template typename Derived can be any dynamically sized matrix type supported by Eigen.
//----------------------------------------
template<typename Derived = Eigen::MatrixXd>
class Parser
{
public:
  // A map to hold named values.
  typedef std::map<std::string, Value<Derived>> ValueMap;

private:
  // Variables are stored in a map of named values.
  ValueMap mVariables;

  // Operator symbols and function names used by the parser.
  std::string mOperators1, mOperators2;
  std::vector<std::string> mFunctions;

  // Expressions are parsed by first splitting them into chunks.
  struct Chunk
  {
    std::string field;
    int type;
    Value<Derived> value;
    int row0, col0, rows, cols;
    Chunk(
      const std::string & str = "", int t = -1,
      const Value<Derived> & val = Value<Derived>())
    : field(str), type(t), value(val),
      row0(-1), col0(-1), rows(-1), cols(-1)
    {
    }
  };
  enum ChunkType { VALUE = 0, VARIABLE, OPERATOR, FUNCTION };
  typedef std::vector<Chunk> ChunkArray;
  typedef typename Derived::Index Index;
  bool mCacheChunkedExpressions;
  std::map<std::string, ChunkArray> mCachedChunkedExpressions;

public:
  // Access to named variables.
  // !!! WARNING! var(name) will create the variable name if it does not already exist.
  inline ValueMap & vars() {return mVariables;}
  inline Value<Derived> & var(const std::string & name)
  {
    return mVariables[name];
  }

  // Check if a variable exists.
  inline bool hasVar(const std::string & name) {return isVariable(name);}

  // Delete a variable.
  inline void clearVar(const std::string & name)
  {
    typename ValueMap::iterator it = mVariables.find(name); if (it != mVariables.end()) {
      mVariables.erase(it);
    }
  }

  // Expression chunk caching.
  inline bool cacheExpressions() const {return mCacheChunkedExpressions;}
  inline void setCacheExpressions(bool b) {mCacheChunkedExpressions = b;}
  inline void clearCachedExpressions() {mCachedChunkedExpressions.clear();}

  Parser();
  ~Parser()
  {
    clearCachedExpressions();
  }

  // Evaluate an expression and return the result in a value wrapper.
  Value<Derived> eval(const std::string & expression);

private:
  void splitEquationIntoChunks(
    const std::string & expression, ChunkArray & chunks,
    std::string & code);
  std::string::const_iterator findClosingBracket(
    const std::string & str,
    const std::string::const_iterator openingBracket,
    const char closingBracket) const;
  std::vector<std::string> splitArguments(const std::string & str, const char delimeter) const;
  void evalIndexRange(const std::string & str, int * first, int * last, int numIndices);
  void evalMatrixExpression(const std::string & str, Value<Derived> & mat);
  void evalFunction(
    const std::string & name, std::vector<std::string> & args,
    Value<Derived> & result);
  bool evalFunction_1_lt(
    const std::string & name, Value<Derived> & arg,
    Value<Derived> & result, std::false_type);
  bool evalFunction_1_lt(
    const std::string & name, Value<Derived> & arg,
    Value<Derived> & result, std::true_type);
  bool evalFunction_2_lt(
    const std::string & name, Value<Derived> & arg0,
    Value<Derived> & arg1, Value<Derived> & result, std::false_type);
  bool evalFunction_2_lt(
    const std::string & name, Value<Derived> & arg0,
    Value<Derived> & arg1, Value<Derived> & result, std::true_type);

  void evalNumericRange(const std::string & str, Value<Derived> & mat);
  inline bool isVariable(const std::string & name) const {return mVariables.count(name) > 0;}
  inline bool isOperator(const char c) const
  {
    return std::find(mOperators1.begin(), mOperators1.end(), c) != mOperators1.end();
  }
  bool isOperator(const std::string & str) const;
  inline bool isFunction(const std::string & str) const
  {
    return std::find(mFunctions.begin(), mFunctions.end(), str) != mFunctions.end();
  }
  void evalIndices(ChunkArray & chunks);
  void evalNegations(ChunkArray & chunks);
  void evalPowers(ChunkArray & chunks);
  void evalMultiplication(ChunkArray & chunks);
  void evalAddition(ChunkArray & chunks);
  void evalAssignment(ChunkArray & chunks);
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  void printChunks(
    ChunkArray & chunks, size_t maxRows = 2, size_t maxCols = 2,
    int precision = 0);
  void printVars(size_t maxRows = 2, size_t maxCols = 2, int precision = 0);
  std::string textRepresentation(
    Value<Derived> & val, size_t maxRows = 2, size_t maxCols = 2,
    int precision = 0);
#       endif
#endif

public:
  static std::string trim(const std::string & str);
  static std::vector<std::string> split(const std::string & str, const char delimeter);
  template<typename T>
  static bool isNumber(const std::string & str, T * num = 0);
  template<typename T>
  static T stringToNumber(const std::string & str);
  template<typename T>
  static std::string numberToString(T num, int precision = 0);
#ifdef DEBUG
  void test_w_lt(
    size_t & numFails, typename Derived::Scalar & s, Derived & a34, Derived & b34,
    Derived & c43, Derived & v, std::true_type);
  void test_w_lt(
    size_t & numFails, typename Derived::Scalar & s, Derived & a34, Derived & b34,
    Derived & c43, Derived & v, std::false_type);
  size_t test();
#endif
};
typedef Parser<Eigen::MatrixXd> ParserXd;
typedef Parser<Eigen::MatrixXf> ParserXf;
typedef Parser<Eigen::MatrixXi> ParserXi;

//----------------------------------------
// Function definitions.
//----------------------------------------
template<typename Derived>
Parser<Derived>::Parser()
: mOperators1("+-*/^()[]="),
  mOperators2(".+.-.*./.^"),
  mCacheChunkedExpressions(false)
{
  // Coefficient-wise operations.
  mFunctions.push_back("abs");
  mFunctions.push_back("sqrt");
  mFunctions.push_back("square");
  mFunctions.push_back("exp");
  mFunctions.push_back("log");
  mFunctions.push_back("log10");
  mFunctions.push_back("sin");
  mFunctions.push_back("cos");
  mFunctions.push_back("tan");
  mFunctions.push_back("asin");
  mFunctions.push_back("acos");

  // Matrix reduction operations.
  mFunctions.push_back("trace");
  mFunctions.push_back("norm");
  mFunctions.push_back("size");
  if (has_operator_lt<typename Derived::Scalar>::value) {
    mFunctions.push_back("min");
    mFunctions.push_back("minOfFinites");
    mFunctions.push_back("max");
    mFunctions.push_back("maxOfFinites");
    mFunctions.push_back("absmax");
    mFunctions.push_back("cwiseMin");
    mFunctions.push_back("cwiseMax");
  }
  mFunctions.push_back("mean");
  mFunctions.push_back("meanOfFinites");
  mFunctions.push_back("sum");
  mFunctions.push_back("sumOfFinites");
  mFunctions.push_back("prod");
  mFunctions.push_back("numberOfFinites");

  // Matrix operations.
  mFunctions.push_back("transpose");
  mFunctions.push_back("conjugate");
  mFunctions.push_back("adjoint");

  // Matrix initializers.
  mFunctions.push_back("zeros");
  mFunctions.push_back("ones");
  mFunctions.push_back("eye");
}

template<typename Derived>
Value<Derived> Parser<Derived>::eval(const std::string & expression)
{
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  std::cout << "---" << std::endl;
  std::cout << "EXPRESSION: " << expression << std::endl;
#       endif
#endif
  ChunkArray chunks;
  std::string code;
  splitEquationIntoChunks(trim(expression), chunks, code);
  evalIndices(chunks);
  evalNegations(chunks);
  evalPowers(chunks);
  evalMultiplication(chunks);
  evalAddition(chunks);
  evalAssignment(chunks);
  if (chunks.size() != 1) {
    throw std::runtime_error(
            "Failed to reduce expression '" + expression + "' to a single value.");
  }
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  std::cout << "---" << std::endl;
#       endif
#endif
  if (chunks[0].type == VARIABLE) {
    if (!isVariable(chunks[0].field)) {
      throw std::runtime_error("Unknown variable '" + chunks[0].field + "'.");
    }
    return mVariables[chunks[0].field];
  }
  return chunks[0].value;
}

template<typename Derived>
void Parser<Derived>::splitEquationIntoChunks(
  const std::string & expression,
  ChunkArray & chunks, std::string & code)
{
  if (cacheExpressions()) {
    if (mCachedChunkedExpressions.count(expression) > 0) {
      chunks = mCachedChunkedExpressions.at(expression);
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
      std::cout << "CACHED CHUNKS: "; printChunks(chunks); std::cout << std::endl;
#       endif
#endif
      return;
    }
  }

  for (std::string::const_iterator it = expression.begin(); it != expression.end(); ) {
    int prevType = (chunks.size() ? chunks.back().type : -1);
    char ci = (*it);
    if (isspace(ci)) {
      // Ignore whitespace.
      it++;
    } else if (ci == '(' && (prevType == VALUE || prevType == VARIABLE)) {
      // Index group.
      std::string::const_iterator jt = findClosingBracket(expression, it, ')');
      if (jt == expression.end()) {
        throw std::runtime_error("Missing closing bracket for '" + std::string(it, jt) + "'.");
      }
      std::string field = std::string(it + 1, jt);    // Outer brackets stripped.
      if (prevType == VARIABLE) {
        if (!isVariable(chunks.back().field)) {
          throw std::runtime_error("Unknown variable '" + chunks.back().field + "'.");
        }
        chunks.back().value.setShared(var(chunks.back().field));
      }
      int first, last;
      int rows = static_cast<int>(chunks.back().value.matrix().rows());
      int cols = static_cast<int>(chunks.back().value.matrix().cols());
      std::vector<std::string> args = splitArguments(field, ',');
      if (args.size() == 1) {
        if (cols == 1) {
          evalIndexRange(args[0], &first, &last, rows);
          chunks.back().row0 = first;
          chunks.back().col0 = 0;
          // (last == -1 ? int(chunks.back().value.matrix().rows()) : last + 1) - first;
          chunks.back().rows = last + 1 - first;
          chunks.back().cols = 1;
        } else if (rows == 1) {
          evalIndexRange(args[0], &first, &last, cols);
          chunks.back().row0 = 0;
          chunks.back().col0 = first;
          chunks.back().rows = 1;
          // (last == -1 ? int(chunks.back().value.matrix().cols()) : last + 1) - first;
          chunks.back().cols = last + 1 - first;
        } else {
          throw std::runtime_error(
                  "Missing row or column indices for '(" + chunks.back().field + "(" + field +
                  ")'.");
        }
      } else if (args.size() == 2) {
        evalIndexRange(args[0], &first, &last, rows);
        chunks.back().row0 = first;
        // (last == -1 ? int(chunks.back().value.matrix().rows()) : last + 1) - first;
        chunks.back().rows = last + 1 - first;
        evalIndexRange(args[1], &first, &last, cols);
        chunks.back().col0 = first;
        // (last == -1 ? int(chunks.back().value.matrix().cols()) : last + 1) - first;
        chunks.back().cols = last + 1 - first;
      } else {
        throw std::runtime_error(
                "Invalid index expression '" + chunks.back().field + "(" + field + ")'.");
      }
      code += "i";
      it = jt + 1;
    } else if (ci == '(' && prevType == FUNCTION) {
      // Function argument group.
      std::string::const_iterator jt = findClosingBracket(expression, it, ')');
      if (jt == expression.end()) {
        throw std::runtime_error("Missing closing bracket for '" + std::string(it, jt) + "'.");
      }
      std::string field = std::string(it + 1, jt);    // Outer brackets stripped.
      std::vector<std::string> args = splitArguments(field, ',');
      evalFunction(chunks.back().field, args, chunks.back().value);
      chunks.back().field += "(" + field + ")";
      chunks.back().type = VALUE;
      code += (chunks.back().value.matrix().size() == 1 ? "n" : "M");
      it = jt + 1;
    } else if (ci == '(') {
      // Recursively evaluate group to a single value.
      std::string::const_iterator jt = findClosingBracket(expression, it, ')');
      if (jt == expression.end()) {
        throw std::runtime_error("Missing closing bracket for '" + std::string(it, jt) + "'.");
      }
      std::string field = std::string(it + 1, jt);    // Outer brackets stripped.
      chunks.push_back(Chunk(field, prevType = VALUE, eval(field)));
      code += (chunks.back().value.matrix().size() == 1 ? "n" : "M");
      it = jt + 1;
    } else if (ci == '[') {
      // Evaluate matrix.
      if (prevType == VALUE || prevType == VARIABLE) {
        throw std::runtime_error(
                "Invalid operation '" + chunks.back().field + std::string(1, ci) + "'.");
      }
      std::string::const_iterator jt = findClosingBracket(expression, it, ']');
      if (jt == expression.end()) {
        throw std::runtime_error("Missing closing bracket for '" + std::string(it, jt) + "'.");
      }
      std::string field = std::string(it + 1, jt);    // Outer brackets stripped.
      chunks.push_back(Chunk("[" + field + "]", prevType = VALUE));
      evalMatrixExpression(field, chunks.back().value);
      code += (chunks.back().value.matrix().size() == 1 ? "n" : "M");
      it = jt + 1;
    } else if (it + 1 != expression.end() && isOperator(std::string(it, it + 2))) {
      // Double character operator.
      std::string field = std::string(it, it + 2);
      chunks.push_back(Chunk(field, prevType = OPERATOR));
      code += field;
      it += 2;
    } else if (isOperator(ci)) {
      // Single character operator.
      std::string field = std::string(1, ci);
      chunks.push_back(Chunk(field, prevType = OPERATOR));
      code += field;
      it++;
    } else {
      // Non-operator: value range, number, function, or variable name.
      std::string::const_iterator jt = it + 1;
      // accept fp-strings, ie [+-]
      unsigned char state = 1;
      for (std::string::const_iterator kt = it; state && kt != expression.end(); kt++) {
        unsigned char token;
        if (*kt == ' ') {token = 0;} else if (*kt == '+' || *kt == '-') {
          token = 1;
        } else if (isdigit(*kt)) {token = 2;} else if (*kt == '.') {
          token = 3;
        } else if (*kt == 'e' || *kt == 'E') {token = 4;} else {break;}
        static const char nextstate[9][5] = {{0},
          {1, 2, 3, 4, 0},
          {0, 0, 3, 4, 0},
          {0, 0, 3, 5, 6},
          {0, 0, 5, 0, 0},
          {0, 0, 5, 0, 6},
          {0, 7, 8, 0, 0},
          {0, 0, 8, 0, 0},
          {0, 0, 8, 0, 0}};
        // WARN("state=" << (int)state << " token(" << *kt << ")=" << (int)token
        //   << " nextstate = " << (int)nextstate[state][token] << "\n");
        state = nextstate[state][token];
        if (state == 8) {jt = kt;}
      }
      for (; jt != expression.end(); jt++) {
        if (isOperator(*jt) ||
          (jt + 1 != expression.end() && isOperator(std::string(jt, jt + 2))))
        {
          break;
        }
      }
      std::string field = trim(std::string(it, jt));
      if (prevType == VALUE || prevType == VARIABLE) {
        throw std::runtime_error("Invalid operation '" + chunks.back().field + field + "'.");
      }
      double num;
      if (field.find(":") != std::string::npos) {
        // Numeric range.
        chunks.push_back(Chunk(field, prevType = VALUE));
        evalNumericRange(field, chunks.back().value);
        code += (chunks.back().value.matrix().size() == 1 ? "n" : "M");
      } else if (isNumber<double>(field, &num)) {
        // Number.
        chunks.push_back(Chunk(field, prevType = VALUE, Value<Derived>(num)));
        code += "n";
      } else if (isVariable(field)) {
        // Local variable.
        chunks.push_back(Chunk(field, prevType = VARIABLE));
        code += (mVariables[field].matrix().size() == 1 ? "vn" : "vM");
      } else if (isFunction(field)) {
        // Function.
        chunks.push_back(Chunk(field, prevType = FUNCTION));
      } else {
        // New undefined variable.
        chunks.push_back(Chunk(field, prevType = VARIABLE));
        code += "vU";
      }
      it = jt;
    }
  }             // it
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  std::cout << "CHUNKS: "; printChunks(chunks); std::cout << std::endl;
  std::cout << "CODE: " << code << std::endl;
#       endif
#endif
  if (cacheExpressions()) {
    mCachedChunkedExpressions[expression] = chunks;
  }
}

template<typename Derived>
std::string::const_iterator Parser<Derived>::findClosingBracket(
  const std::string & str,
  const std::string::const_iterator openingBracket,
  const char closingBracket) const
{
  int depth = 1;
  std::string::const_iterator it = openingBracket + 1;
  for (; it != str.end(); it++) {
    if ((*it) == (*openingBracket)) {depth++;} else if ((*it) == closingBracket) {depth--;}
    if (depth == 0) {return it;}
  }
  return str.end();
}

template<typename Derived>
std::vector<std::string> Parser<Derived>::splitArguments(
  const std::string & str,
  const char delimeter) const
{
  std::vector<std::string> args;
  std::string::const_iterator i0 = str.begin();
  for (std::string::const_iterator it = str.begin(); it != str.end(); it++) {
    if ((*it) == '(') {it = findClosingBracket(str, it, ')');} else if ((*it) == '[') {
      it = findClosingBracket(str, it, ']');
    } else if ((*it) == delimeter) {
      args.push_back(trim(std::string(i0, it)));
      i0 = it + 1;
    }
  }
  args.push_back(std::string(i0, str.end()));
  return args;
}

template<typename Derived>
void Parser<Derived>::evalIndexRange(
  const std::string & str, int * first, int * last,
  int numIndices)
{
  if (str.empty()) {
    throw std::runtime_error("Empty index range.");
  }
  ValueXi valuei;
  ParserXi parseri;
  size_t pos;
  for (std::string::const_iterator it = str.begin(); it != str.end(); it++) {
    if ((*it) == ':') {
      std::string firstStr = trim(std::string(str.begin(), it));
      std::string lastStr = trim(std::string(it + 1, str.end()));
      if (firstStr.empty() && lastStr.empty()) {
        (*first) = 0;
        (*last) = numIndices - 1;
        return;
      }
      if (firstStr.empty() || lastStr.empty()) {
        throw std::runtime_error("Missing indices for '" + str + "'.");
      }

      pos = firstStr.find("end");
      if (pos != std::string::npos) {
        firstStr =
          firstStr.substr(0, pos) + numberToString<int>(numIndices - 1) + firstStr.substr(
          pos + 3);
      }
      pos = lastStr.find("end");
      if (pos != std::string::npos) {
        lastStr =
          lastStr.substr(0, pos) + numberToString<int>(numIndices - 1) + lastStr.substr(
          pos + 3);
      }

      valuei = parseri.eval(firstStr);
      if (valuei.matrix().size() != 1) {
        throw std::runtime_error("Invalid indices '" + str + "'.");
      }
      (*first) = valuei.matrix()(0, 0);

      valuei = parseri.eval(lastStr);
      if (valuei.matrix().size() != 1) {
        throw std::runtime_error("Invalid indices '" + str + "'.");
      }
      (*last) = valuei.matrix()(0, 0);

      return;
    }
  }
  std::string firstStr = str;

  pos = firstStr.find("end");
  if (pos != std::string::npos) {
    firstStr =
      firstStr.substr(
      0,
      pos) + numberToString<int>(numIndices - 1) + firstStr.substr(pos + 3);
  }

  valuei = parseri.eval(firstStr);
  if (valuei.matrix().size() != 1) {
    throw std::runtime_error("Invalid index '" + str + "'.");
  }
  (*first) = valuei.matrix()(0, 0);
  (*last) = (*first);
}

template<typename Derived>
void Parser<Derived>::evalMatrixExpression(const std::string & str, Value<Derived> & mat)
{
  // !!! Expression may NOT include outer brackets, although brackets for individual rows are OK.
  std::vector<std::string> rows = splitArguments(str, ';');
  std::vector<std::vector<typename Derived::Scalar>> temp;
  Value<Derived> submatrix;
  size_t row0 = 0, col0 = 0, nrows = 0, ncols = 0;
  size_t pos;
  for (size_t i = 0; i < rows.size(); i++) {
    // Strip row brackets if they exist.
    if (rows[i][0] == '[' && rows[i].back() == ']') {
      rows[i] = rows[i].substr(1, static_cast<int>(rows[i].size()) - 2);
    }
    std::vector<std::string> cols = splitArguments(rows[i], ',');
    col0 = 0;
    ncols = 0;
    for (size_t j = 0; j < cols.size(); j++) {
      pos = cols[j].find(":");
      if (pos != std::string::npos) {
        std::string firstStr = cols[j].substr(0, pos);
        std::string lastStr = cols[j].substr(pos + 1);
        pos = lastStr.find(":");
        if (pos != std::string::npos) {
          std::string stepStr = lastStr.substr(0, pos);
          lastStr = lastStr.substr(pos + 1);
          if (lastStr.find(":") != std::string::npos) {
            throw std::runtime_error(
                    "Invalid matrix definition '[" + str + "]'. Invalid range '" + cols[j] + "'.");
          }
          // first:step:last
          Value<Derived> first = eval(firstStr);
          Value<Derived> step = eval(stepStr);
          Value<Derived> last = eval(lastStr);
          if (first.matrix().size() != 1 || step.matrix().size() != 1 ||
            last.matrix().size() != 1)
          {
            throw std::runtime_error(
                    "Invalid matrix definition '[" + str + "]'. Invalid range '" + cols[j] + "'.");
          }
          typename Derived::RealScalar sfirst = std::real(first.matrix()(0));
          typename Derived::RealScalar sstep = std::real(step.matrix()(0));
          typename Derived::RealScalar slast = std::real(last.matrix()(0));
          if (sfirst == slast) {
            submatrix.local().setConstant(1, 1, sfirst);
            submatrix.mapLocal();
          } else if ((slast - sfirst >= 0 && sstep > 0) || (slast - sfirst <= 0 && sstep < 0)) {
            int n = floor((slast - sfirst) / sstep) + 1;
            submatrix.local().resize(1, n);
            for (int k = 0; k < n; ++k) {
              submatrix.local()(0, k) = sfirst + k * sstep;
            }
            submatrix.mapLocal();
          } else {
            throw std::runtime_error(
                    "Invalid matrix definition '[" + str + "]'. Invalid range '" + cols[j] + "'.");
          }
        } else {
          // first:last => first:1:last
          Value<Derived> first = eval(firstStr);
          Value<Derived> last = eval(lastStr);
          if (first.matrix().size() != 1 || last.matrix().size() != 1) {
            throw std::runtime_error(
                    "Invalid matrix definition '[" + str + "]'. Invalid range '" + cols[j] + "'.");
          }
          typename Derived::RealScalar sfirst = std::real(first.matrix()(0));
          typename Derived::RealScalar slast = std::real(last.matrix()(0));
          if (sfirst == slast) {
            submatrix.local().setConstant(1, 1, sfirst);
            submatrix.mapLocal();
          } else if (slast - sfirst >= 0) {
            int n = floor(slast - sfirst) + 1;
            submatrix.local().resize(1, n);
            for (int k = 0; k < n; ++k) {
              submatrix.local()(0, k) = sfirst + k;
            }
            submatrix.mapLocal();
          } else {
            throw std::runtime_error(
                    "Invalid matrix definition '[" + str + "]'. Invalid range '" + cols[j] + "'.");
          }
        }
      } else {
        submatrix = eval(cols[j]);
      }
      if (j > 0 && size_t(submatrix.matrix().cols()) != nrows) {
        throw std::runtime_error(
                "Invalid matrix definition '[" + str + "]'. Successive column entries '" +
                cols[static_cast<int>(j) - 1] + "' and '" + cols[j] +
                "' do not have the same number of rows.");
      }
      nrows = submatrix.matrix().rows();
      ncols += submatrix.matrix().cols();
      temp.resize(row0 + submatrix.matrix().rows());
      for (size_t row = 0; row < size_t(submatrix.matrix().rows()); row++) {
        temp[row0 + row].resize(col0 + submatrix.matrix().cols());
        for (size_t col = 0; col < size_t(submatrix.matrix().cols()); col++) {
          temp[row0 + row][col0 + col] = submatrix.matrix()(row, col);
        }
      }
      col0 += submatrix.matrix().cols();
    }
    if (row0 > 0 && ncols != temp[static_cast<int>(row0) - 1].size()) {
      throw std::runtime_error(
              "Invalid matrix definition '[" + str + "]'. Successive row entries '" +
              rows[static_cast<int>(i) - 1] + "' and '" + rows[i] +
              "' do not have the same number of columns.");
    }
    row0 += nrows;
  }
  if (temp.empty()) {return;}
  nrows = temp.size();
  ncols = temp[0].size();
  mat.setLocal(Derived(nrows, ncols));
  for (size_t row = 0; row < nrows; row++) {
    for (size_t col = 0; col < ncols; col++) {
      mat.matrix()(row, col) = temp[row][col];
    }
  }
  mat.mapLocal();
}

template<typename Derived>
bool Parser<Derived>::evalFunction_1_lt(
  const std::string & name, Value<Derived> & arg,
  Value<Derived> & result, std::true_type)
{
  if (name == "min") {
    result.setLocal(arg.matrix().minCoeff());
    return true;
  } else if (name == "minOfFinites") {
    result.setLocal(arg.matrix().minCoeffOfFinites());
    return true;
  } else if (name == "max") {
    result.setLocal(arg.matrix().maxCoeff());
    return true;
  } else if (name == "maxOfFinites") {
    result.setLocal(arg.matrix().maxCoeffOfFinites());
    return true;
  } else if (name == "absmax") {
    typename Derived::Scalar minimum = arg.matrix().minCoeff();
    typename Derived::Scalar maximum = arg.matrix().maxCoeff();
    result.setLocal(std::abs(maximum) >= std::abs(minimum) ? maximum : minimum);
    return true;
  }
  return false;
}

template<typename Derived>
bool Parser<Derived>::evalFunction_1_lt(
  const std::string & /*name*/,
  Value<Derived> & /*arg0*/,
  Value<Derived> & /*result*/, std::false_type)
{
  return false;
}

template<typename Derived>
bool Parser<Derived>::evalFunction_2_lt(
  const std::string & name, Value<Derived> & arg0,
  Value<Derived> & arg1, Value<Derived> & result,
  std::true_type)
{
  if (name == "min") {
    if (arg1.matrix().size() != 1) {
      throw std::runtime_error("Invalid dimension argument for function '" + name + "(...)'.");
    }
    int dim = floor(std::real(arg1.matrix()(0, 0)));
    if ((dim != 0 && dim != 1) || dim != std::real(arg1.matrix()(0, 0))) {
      throw std::runtime_error("Invalid dimension argument for function '" + name + "(...)'.");
    }
    if (dim == 0) {
      result.local() = arg0.matrix().colwise().minCoeff();
      result.mapLocal();
      return true;
    } else if (dim == 1) {
      result.local() = arg0.matrix().rowwise().minCoeff();
      result.mapLocal();
      return true;
    }
  } else if (name == "max") {
    if (arg1.matrix().size() != 1) {
      throw std::runtime_error("Invalid dimension argument for function '" + name + "(...)'.");
    }
    int dim = floor(std::real(arg1.matrix()(0, 0)));
    if ((dim != 0 && dim != 1) || dim != std::real(arg1.matrix()(0, 0))) {
      throw std::runtime_error("Invalid dimension argument for function '" + name + "(...)'.");
    }
    if (dim == 0) {
      result.local() = arg0.matrix().colwise().maxCoeff();
      result.mapLocal();
      return true;
    } else if (dim == 1) {
      result.local() = arg0.matrix().rowwise().maxCoeff();
      result.mapLocal();
      return true;
    }
  } else if (name == "absmax") {
    if (arg1.matrix().size() != 1) {
      throw std::runtime_error("Invalid dimension argument for function '" + name + "(...)'.");
    }
    int dim = floor(std::real(arg1.matrix()(0, 0)));
    if ((dim != 0 && dim != 1) || dim != std::real(arg1.matrix()(0, 0))) {
      throw std::runtime_error("Invalid dimension argument for function '" + name + "(...)'.");
    }
    if (dim == 0) {
      result.local() = arg0.matrix().colwise().maxCoeff();
      result.mapLocal();
      Derived minimum = arg0.matrix().colwise().minCoeff();
      for (size_t i = 0; i < size_t(result.matrix().size()); i++) {
        if (std::abs(result.matrix()(i)) < std::abs(minimum(i))) {
          result.matrix()(i) = minimum(i);
        }
      }
      return true;
    } else if (dim == 1) {
      result.local() = arg0.matrix().rowwise().maxCoeff();
      result.mapLocal();
      Derived minimum = arg0.matrix().rowwise().minCoeff();
      for (size_t i = 0; i < size_t(result.matrix().size()); i++) {
        if (std::abs(result.matrix()(i)) < std::abs(minimum(i))) {
          result.matrix()(i) = minimum(i);
        }
      }
      return true;
    }
  } else if (name == "cwiseMin") {
    if (arg1.matrix().size() == 1) {
      typename Derived::Scalar arg1scalar = arg1.matrix()(0, 0);
      Derived arg1matrix = Derived::Constant(
        arg0.matrix().rows(),
        arg0.matrix().cols(), arg1scalar);
      result.local() = arg0.matrix().cwiseMin(arg1matrix);
      result.mapLocal();
      return true;
    } else if (  // NOLINT
      arg0.matrix().cols() == arg1.matrix().cols() &&
      arg0.matrix().rows() == arg1.matrix().rows())
    {
      result.local() = arg0.matrix().cwiseMin(arg1.matrix());
      result.mapLocal();
      return true;
    } else {
      throw std::runtime_error("Invalid dimension argument for function '" + name + "(...)'.");
    }
  } else if (name == "cwiseMax") {
    if (arg1.matrix().size() == 1) {
      typename Derived::Scalar arg1scalar = arg1.matrix()(0, 0);
      Derived arg1matrix = Derived::Constant(
        arg0.matrix().rows(),
        arg0.matrix().cols(), arg1scalar);
      result.local() = arg0.matrix().cwiseMax(arg1matrix);
      result.mapLocal();
      return true;
    } else if (  // NOLINT
      arg0.matrix().cols() == arg1.matrix().cols() &&
      arg0.matrix().rows() == arg1.matrix().rows())
    {
      result.local() = arg0.matrix().cwiseMax(arg1.matrix());
      result.mapLocal();
      return true;
    } else {
      throw std::runtime_error("Invalid dimension argument for function '" + name + "(...)'.");
    }
  }
  return false;
}

template<typename Derived>
bool Parser<Derived>::evalFunction_2_lt(
  const std::string & /*name*/,
  Value<Derived> & /*arg0*/,
  Value<Derived> & /*arg1*/,
  Value<Derived> & /*result*/, std::false_type)
{
  return false;
}

template<typename Derived>
void Parser<Derived>::evalFunction(
  const std::string & name,
  std::vector<std::string> & args,
  Value<Derived> & result)
{
  if (args.size() == 1) {
    Value<Derived> arg = eval(args[0]);
    if (name == "abs") {
      result.local() = arg.matrix().array().abs().template cast<typename Derived::Scalar>();
      result.mapLocal();
      return;
    } else if (name == "sqrt") {
      result.local() = arg.matrix().array().sqrt();
      result.mapLocal();
      return;
    } else if (name == "square") {
      result.local() = arg.matrix().array().square();
      result.mapLocal();
      return;
    } else if (name == "exp") {
      result.local() = arg.matrix().array().exp();
      result.mapLocal();
      return;
    } else if (name == "log") {
      result.local() = arg.matrix().array().log();
      result.mapLocal();
      return;
    } else if (name == "log10") {
      result.local() = arg.matrix().array().log();
      result.local() *= (1.0 / log(10));
      result.mapLocal();
      return;
    } else if (name == "sin") {
      result.local() = arg.matrix().array().sin();
      result.mapLocal();
      return;
    } else if (name == "cos") {
      result.local() = arg.matrix().array().cos();
      result.mapLocal();
      return;
    } else if (name == "tan") {
      result.local() = arg.matrix().array().tan();
      result.mapLocal();
      return;
    } else if (name == "acos") {
      result.local() = arg.matrix().array().acos();
      result.mapLocal();
      return;
    } else if (name == "asin") {
      result.local() = arg.matrix().array().asin();
      result.mapLocal();
      return;
    } else if (name == "trace") {
      result.setLocal(arg.matrix().trace());
      return;
    } else if (name == "norm") {
      result.setLocal(arg.matrix().norm());
      return;
    } else if (  // NOLINT
      evalFunction_1_lt(
        name, arg, result,
        has_operator_lt<typename Derived::Scalar>()))
    {
      return;
    } else if (name == "mean") {
      result.setLocal(arg.matrix().mean());
      return;
    } else if (name == "meanOfFinites") {
      result.setLocal(arg.matrix().meanOfFinites());
      return;
    } else if (name == "sum") {
      result.setLocal(arg.matrix().sum());
      return;
    } else if (name == "sumOfFinites") {
      result.setLocal(arg.matrix().sumOfFinites());
      return;
    } else if (name == "prod") {
      result.setLocal(arg.matrix().prod());
      return;
    } else if (name == "numberOfFinites") {
      result.setLocal(arg.matrix().numberOfFinites());
      return;
    } else if (name == "transpose") {
      result.local() = arg.matrix().transpose();
      result.mapLocal();
      return;
    } else if (name == "conjugate") {
      result.local() = arg.matrix().conjugate();
      result.mapLocal();
      return;
    } else if (name == "adjoint") {
      result.local() = arg.matrix().adjoint();
      result.mapLocal();
      return;
    } else if (name == "zeros") {
      if (arg.matrix().size() != 1) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + ")'.");
      }
      int N = floor(std::real(arg.matrix()(0, 0)));
      if (N <= 0 || N != std::real(arg.matrix()(0, 0))) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + ")'.");
      }
      result.local() = Derived::Zero(N, N);
      result.mapLocal();
      return;
    } else if (name == "ones") {
      if (arg.matrix().size() != 1) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + ")'.");
      }
      int N = floor(std::real(arg.matrix()(0, 0)));
      if (N <= 0 || N != std::real(arg.matrix()(0, 0))) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + ")'.");
      }
      result.local() = Derived::Ones(N, N);
      result.mapLocal();
      return;
    } else if (name == "eye") {
      if (arg.matrix().size() != 1) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + ")'.");
      }
      int N = floor(std::real(arg.matrix()(0, 0)));
      if (N <= 0 || N != std::real(arg.matrix()(0, 0))) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + ")'.");
      }
      result.local() = Derived::Identity(N, N);
      result.mapLocal();
      return;
    } else {
      throw std::runtime_error(
              "Invalid function '" +
              name + "(" + args[0] + ")'.");
    }
  } else if (args.size() == 2) {
    Value<Derived> arg0 = eval(args[0]);
    Value<Derived> arg1 = eval(args[1]);
    if (name == "size") {
      if (arg1.matrix().size() != 1) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      int dim = floor(std::real(arg1.matrix()(0, 0)));
      if ((dim != 0 && dim != 1) || dim != std::real(arg1.matrix()(0, 0))) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      if (dim == 0) {
        result.setLocal((typename Derived::Scalar)arg0.matrix().rows());
        return;
      } else if (dim == 1) {
        result.setLocal((typename Derived::Scalar)arg0.matrix().cols());
        return;
      }
    } else if (  // NOLINT
      evalFunction_2_lt(
        name, arg0, arg1, result,
        has_operator_lt<typename Derived::Scalar>()))
    {
      return;
    } else if (name == "mean") {
      if (arg1.matrix().size() != 1) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      int dim = floor(std::real(arg1.matrix()(0, 0)));
      if ((dim != 0 && dim != 1) || dim != std::real(arg1.matrix()(0, 0))) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      if (dim == 0) {
        result.local() = arg0.matrix().colwise().mean();
        result.mapLocal();
        return;
      } else if (dim == 1) {
        result.local() = arg0.matrix().rowwise().mean();
        result.mapLocal();
        return;
      }
    } else if (name == "sum") {
      if (arg1.matrix().size() != 1) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      int dim = floor(std::real(arg1.matrix()(0, 0)));
      if ((dim != 0 && dim != 1) || dim != std::real(arg1.matrix()(0, 0))) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      if (dim == 0) {
        result.local() = arg0.matrix().colwise().sum();
        result.mapLocal();
        return;
      } else if (dim == 1) {
        result.local() = arg0.matrix().rowwise().sum();
        result.mapLocal();
        return;
      }
    } else if (name == "prod") {
      if (arg1.matrix().size() != 1) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      int dim = floor(std::real(arg1.matrix()(0, 0)));
      if ((dim != 0 && dim != 1) || dim != std::real(arg1.matrix()(0, 0))) {
        throw std::runtime_error(
                "Invalid dimension argument for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      if (dim == 0) {
        result.local() = arg0.matrix().colwise().prod();
        result.mapLocal();
        return;
      } else if (dim == 1) {
        result.local() = arg0.matrix().rowwise().prod();
        result.mapLocal();
        return;
      }
    } else if (name == "zeros") {
      if ((arg0.matrix().size() != 1) || (arg1.matrix().size() != 1)) {
        throw std::runtime_error(
                "Invalid dimension arguments for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      int rows = floor(std::real(arg0.matrix()(0, 0)));
      int cols = floor(std::real(arg1.matrix()(0, 0)));
      if (rows <= 0 || cols <= 0 ||
        rows != std::real(arg0.matrix()(0, 0)) || cols != std::real(arg1.matrix()(0, 0)))
      {
        throw std::runtime_error(
                "Invalid dimension arguments for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      result.local() = Derived::Zero(rows, cols);
      result.mapLocal();
      return;
    } else if (name == "ones") {
      if ((arg0.matrix().size() != 1) || (arg1.matrix().size() != 1)) {
        throw std::runtime_error(
                "Invalid dimension arguments for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      int rows = floor(std::real(arg0.matrix()(0, 0)));
      int cols = floor(std::real(arg1.matrix()(0, 0)));
      if (rows <= 0 || cols <= 0 ||
        rows != std::real(arg0.matrix()(0, 0)) || cols != std::real(arg1.matrix()(0, 0)))
      {
        throw std::runtime_error(
                "Invalid dimension arguments for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      result.local() = Derived::Ones(rows, cols);
      result.mapLocal();
      return;
    } else if (name == "eye") {
      if ((arg0.matrix().size() != 1) || (arg1.matrix().size() != 1)) {
        throw std::runtime_error(
                "Invalid dimension arguments for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      int rows = floor(std::real(arg0.matrix()(0, 0)));
      int cols = floor(std::real(arg1.matrix()(0, 0)));
      if (rows <= 0 || cols <= 0 ||
        rows != std::real(arg0.matrix()(0, 0)) || cols != std::real(arg1.matrix()(0, 0)))
      {
        throw std::runtime_error(
                "Invalid dimension arguments for function '" +
                name + "(" + args[0] + "," + args[1] + ")'.");
      }
      result.local() = Derived::Identity(rows, cols);
      result.mapLocal();
      return;
    } else {
      throw std::runtime_error(
              "Invalid function '" + name + "(" + args[0] + "," + args[1] + ")'.");
    }
  }
  std::string argsStr = "(";
  for (size_t i = 0; i < args.size(); i++) {
    if (i > 0) {argsStr += ",";}
    argsStr += args[i];
  }
  argsStr += ")";
  throw std::runtime_error("Invalid function/arguments for '" + name + argsStr + "'.");
}

template<typename Derived>
void Parser<Derived>::evalNumericRange(const std::string & str, Value<Derived> & mat)
{
  size_t pos = str.find(":");
  if (pos == std::string::npos) {
    throw std::runtime_error("Invalid numeric range '" + str + "'.");
  }
  size_t pos2 = str.substr(pos + 1).find(":");
  if (pos2 == std::string::npos) {
    // first:last
    std::string firstStr = str.substr(0, pos);
    std::string lastStr = str.substr(pos + 1);
    Value<Derived> first = eval(firstStr);
    Value<Derived> last = eval(lastStr);
    if (first.matrix().size() != 1 || last.matrix().size() != 1) {
      throw std::runtime_error("Invalid numeric range '" + str + "'.");
    }
    typename Derived::RealScalar sfirst = std::real(first.matrix()(0, 0));
    typename Derived::RealScalar slast = std::real(last.matrix()(0, 0));
    if (sfirst > slast) {
      throw std::runtime_error("Invalid numeric range '" + str + "'. Must not reverse.");
    }
    int n = 1 + floor(slast - sfirst);
    mat.local().resize(1, n);
    for (int i = 0; i < n; i++) {
      mat.local()(0, i) = sfirst + i;
    }
    mat.mapLocal();
  } else {
    // first:step:last
    pos2 += pos + 1;
    std::string firstStr = str.substr(0, pos);
    std::string stepStr = str.substr(pos + 1, pos2 - pos - 1);
    std::string lastStr = str.substr(pos2 + 1);
    Value<Derived> first = eval(firstStr);
    Value<Derived> step = eval(stepStr);
    Value<Derived> last = eval(lastStr);
    if (first.matrix().size() != 1 || step.matrix().size() != 1 || last.matrix().size() != 1) {
      throw std::runtime_error("Invalid numeric range '" + str + "'.");
    }
    typename Derived::RealScalar sfirst = std::real(first.matrix()(0, 0));
    typename Derived::RealScalar sstep = std::real(step.matrix()(0, 0));
    typename Derived::RealScalar slast = std::real(last.matrix()(0, 0));
    if (sfirst == slast) {
      mat = sfirst;
    } else if (sfirst < slast && sstep > 0) {
      int n = 1 + floor((slast - sfirst) / sstep);
      mat.local().resize(1, n);
      for (int i = 0; i < n; i++) {
        mat.local()(0, i) = sfirst + i * sstep;
      }
      mat.mapLocal();
    } else if (sfirst > slast && sstep < 0) {
      int n = 1 + floor((slast - sfirst) / sstep);
      mat.local().resize(1, n);
      for (int i = 0; i < n; i++) {
        mat.local()(0, i) = sfirst + i * sstep;
      }
      mat.mapLocal();
    } else {
      throw std::runtime_error("Invalid numeric range '" + str + "'.");
    }
  }
}

template<typename Derived>
bool Parser<Derived>::isOperator(const std::string & str) const
{
  if (str.size() == 1) {
    return isOperator(str[0]);
  } else if (str.size() == 2) {
    size_t pos = mOperators2.find(str);
    return pos != std::string::npos && pos % 2 == 0;
  }
  return false;
}

template<typename Derived>
void Parser<Derived>::evalIndices(ChunkArray & chunks)
{
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  bool operationPerformed = false;
#       endif
#endif
  for (typename ChunkArray::iterator it = chunks.begin(); it != chunks.end(); it++) {
    if (it->row0 != -1 &&
      (it->type == VALUE ||
      (it->type == VARIABLE &&
      (it + 1 == chunks.end() || (it + 1)->type != OPERATOR || (it + 1)->field != "="))))
    {
      if (it->type == VALUE) {
        Derived temp = it->value.local().block(it->row0, it->col0, it->rows, it->cols);
        it->value.local() = temp;
        it->value.mapLocal();
      } else {    // if(it->type == VARIABLE) {
        if (!isVariable(it->field)) {
          throw std::runtime_error(
                  "Attempted indexing into uninitialized variable '" + it->field + "'.");
        }
        it->value.local() = mVariables[it->field].matrix().block(
          it->row0, it->col0, it->rows,
          it->cols);
        it->value.mapLocal();
        it->type = VALUE;
      }
      it->row0 = -1;
      it->col0 = -1;
      it->rows = -1;
      it->cols = -1;
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
      operationPerformed = true;
#       endif
#endif
    }
  }
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  if (operationPerformed) {std::cout << "i: "; printChunks(chunks); std::cout << std::endl;}
#       endif
#endif
}

template<typename Derived>
void Parser<Derived>::evalNegations(ChunkArray & chunks)
{
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  bool operationPerformed = false;
#       endif
#endif
  if (chunks.size() < 2) {return;}
  typename ChunkArray::iterator lhs = chunks.begin(), op = chunks.begin(), rhs = op + 1;
  for (; lhs != chunks.end() && op != chunks.end() && rhs != chunks.end(); ) {
    if (op->type == OPERATOR && op->field == "-" &&
      (op == chunks.begin() || (lhs->type != VALUE && lhs->type != VARIABLE)) &&
      (rhs->type == VALUE || rhs->type == VARIABLE))
    {
      if (rhs->type == VALUE) {
        rhs->value.matrix().array() *= -1;
      } else if (rhs->type == VARIABLE) {
        if (!isVariable(rhs->field)) {
          throw std::runtime_error(
                  "Attempted operation '" + op->field + rhs->field +
                  "' on uninitialized variable '" + rhs->field + "'.");
        }
        rhs->value.local() = mVariables[rhs->field].matrix().array() * -1;
        rhs->value.mapLocal();
        rhs->type = VALUE;
      }
      lhs = chunks.erase(op);
      op = (lhs != chunks.end()) ? lhs + 1 : lhs;
      rhs = (op != chunks.end()) ? op + 1 : op;
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
      operationPerformed = true;
#       endif
#endif
    } else {
      lhs = op;
      op = rhs;
      rhs++;
    }
  }
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  if (operationPerformed) {std::cout << "-: "; printChunks(chunks); std::cout << std::endl;}
#       endif
#endif
}

template<typename Derived>
void Parser<Derived>::evalPowers(ChunkArray & chunks)
{
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  bool operationPerformed = false;
#       endif
#endif
  if (chunks.size() < 3) {return;}
  typename ChunkArray::iterator lhs = chunks.begin(), op = lhs + 1, rhs = op + 1;
  for (; lhs != chunks.end() && op != chunks.end() && rhs != chunks.end(); ) {
    if ((op->type == OPERATOR) && (op->field == "^" || op->field == ".^")) {
      if (lhs->type == VARIABLE) {
        if (!isVariable(lhs->field)) {
          throw std::runtime_error(
                  "Attempted operation '" + lhs->field + op->field + rhs->field +
                  "' on uninitialized variable '" + lhs->field + "'.");
        }
        lhs->value.setShared(mVariables[lhs->field]);
      }
      if (rhs->type == VARIABLE) {
        if (!isVariable(rhs->field)) {
          throw std::runtime_error(
                  "Attempted operation '" + lhs->field + op->field + rhs->field +
                  "' on uninitialized variable '" + rhs->field + "'.");
        }
        rhs->value.setShared(mVariables[rhs->field]);
      }
      if (rhs->value.matrix().size() == 1) {
        lhs->value.local() = lhs->value.matrix().array().pow(rhs->value.matrix()(0, 0));
        lhs->value.mapLocal();
        lhs->type = VALUE;
      } else if (lhs->value.matrix().size() == 1) {
        typename Derived::Scalar temp = lhs->value.matrix()(0, 0);
        lhs->value.local().resize(rhs->value.matrix().rows(), rhs->value.matrix().cols());
        for (size_t row = 0; row < size_t(rhs->value.matrix().rows()); row++) {
          for (size_t col = 0; col < size_t(rhs->value.matrix().cols()); col++) {
            lhs->value.local()(row, col) = pow(temp, rhs->value.matrix()(row, col));
          }
        }
        lhs->value.mapLocal();
        lhs->type = VALUE;
      } else if (  // NOLINT
        op->field == ".^" && lhs->value.matrix().rows() == rhs->value.matrix().rows() &&
        lhs->value.matrix().cols() == rhs->value.matrix().cols())
      {
        lhs->value.local().resize(rhs->value.matrix().rows(), rhs->value.matrix().cols());
        for (size_t row = 0; row < size_t(rhs->value.matrix().rows()); row++) {
          for (size_t col = 0; col < size_t(rhs->value.matrix().cols()); col++) {
            lhs->value.local()(
              row,
              col) = pow(lhs->value.matrix()(row, col), rhs->value.matrix()(row, col));
          }
        }
        lhs->value.mapLocal();
        lhs->type = VALUE;
      } else {
        throw std::runtime_error(
                "Invalid operand dimensions for operation '" +
                lhs->field + op->field + rhs->field + "'.");
      }
      chunks.erase(op, rhs + 1);
      op = lhs + 1;
      rhs = (op != chunks.end()) ? op + 1 : op;
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
      operationPerformed = true;
#       endif
#endif
    } else {
      lhs = op;
      op = rhs;
      rhs++;
    }
  }
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  if (operationPerformed) {std::cout << "^: "; printChunks(chunks); std::cout << std::endl;}
#       endif
#endif
}

template<typename Derived>
void Parser<Derived>::evalMultiplication(ChunkArray & chunks)
{
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  bool operationPerformed = false;
#       endif
#endif
  if (chunks.size() < 3) {return;}
  typename ChunkArray::iterator lhs = chunks.begin(), op = lhs + 1, rhs = op + 1;
  for (; lhs != chunks.end() && op != chunks.end() && rhs != chunks.end(); ) {
    if ((op->type == OPERATOR) &&
      (op->field == "*" || op->field == "/" || op->field == ".*" || op->field == "./"))
    {
      if (lhs->type == VARIABLE) {
        if (!isVariable(lhs->field)) {
          throw std::runtime_error(
                  "Attempted operation '" + lhs->field + op->field + rhs->field +
                  "' on uninitialized variable '" + lhs->field + "'.");
        }
        lhs->value.setShared(mVariables[lhs->field]);
      }
      if (rhs->type == VARIABLE) {
        if (!isVariable(rhs->field)) {
          throw std::runtime_error(
                  "Attempted operation '" + lhs->field + op->field + rhs->field +
                  "' on uninitialized variable '" + rhs->field + "'.");
        }
        rhs->value.setShared(mVariables[rhs->field]);
      }
      if (rhs->value.matrix().size() == 1) {
        if (lhs->value.isLocal()) {
          if (op->field == "*" || op->field == ".*") {
            lhs->value.local().array() *= rhs->value.matrix()(0, 0);
          } else {                                 // if(op->field == "/" || op->field == "./")
            lhs->value.local().array() /= rhs->value.matrix()(0, 0);
          }
        } else {
          if (op->field == "*" || op->field == ".*") {
            lhs->value.local() = lhs->value.matrix().array() * rhs->value.matrix()(0, 0);
          } else {                                 // if(op->field == "/" || op->field == "./")
            lhs->value.local() = lhs->value.matrix().array() / rhs->value.matrix()(0, 0);
          }
          lhs->value.mapLocal();
          lhs->type = VALUE;
        }
      } else if (lhs->value.matrix().size() == 1) {
        typename Derived::Scalar temp = lhs->value.matrix()(0, 0);
        if (op->field == "*" || op->field == ".*") {
          lhs->value.local() = rhs->value.matrix().array() * temp;
        } else {                           // if(op->field == "/" || op->field == "./")
          lhs->value.local() = Derived::Constant(
            rhs->value.matrix().rows(),
            rhs->value.matrix().cols(), temp).array() / rhs->value.matrix().array();
        }
        lhs->value.mapLocal();
        lhs->type = VALUE;
      } else if (  // NOLINT
        (op->field == ".*" || op->field == "./") &&
        lhs->value.matrix().rows() == rhs->value.matrix().rows() &&
        lhs->value.matrix().cols() == rhs->value.matrix().cols())
      {
        if (lhs->value.isLocal()) {
          if (op->field == ".*") {
            lhs->value.local().array() *= rhs->value.matrix().array();
          } else {                                 // if(op->field == "./")
            lhs->value.local().array() /= rhs->value.matrix().array();
          }
        } else {
          if (op->field == ".*") {
            lhs->value.local() = lhs->value.matrix().array() * rhs->value.matrix().array();
          } else {                                 // if(op->field == "./")
            lhs->value.local() = lhs->value.matrix().array() / rhs->value.matrix().array();
          }
          lhs->value.mapLocal();
          lhs->type = VALUE;
        }
      } else if (op->field == "*" && lhs->value.matrix().cols() == rhs->value.matrix().rows()) {
        if (lhs->value.isLocal()) {
          lhs->value.local() *= rhs->value.matrix();
          lhs->value.mapLocal();
        } else {
          lhs->value.local() = lhs->value.matrix() * rhs->value.matrix();
          lhs->value.mapLocal();
          lhs->type = VALUE;
        }
      } else {
        throw std::runtime_error(
                "Invalid operand dimensions for operation '" +
                lhs->field + op->field + rhs->field + "'.");
      }
      chunks.erase(op, rhs + 1);
      op = lhs + 1;
      rhs = (op != chunks.end()) ? op + 1 : op;
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
      operationPerformed = true;
#       endif
#endif
    } else {
      lhs = op;
      op = rhs;
      rhs++;
    }
  }
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  if (operationPerformed) {std::cout << "*: "; printChunks(chunks); std::cout << std::endl;}
#       endif
#endif
}

template<typename Derived>
void Parser<Derived>::evalAddition(ChunkArray & chunks)
{
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  bool operationPerformed = false;
#       endif
#endif
  if (chunks.size() < 3) {return;}
  typename ChunkArray::iterator lhs = chunks.begin(), op = lhs + 1, rhs = op + 1;
  for (; lhs != chunks.end() && op != chunks.end() && rhs != chunks.end(); ) {
    if ((op->type == OPERATOR) &&
      (op->field == "+" || op->field == "-" || op->field == ".+" || op->field == ".-"))
    {
      if (lhs->type == VARIABLE) {
        if (!isVariable(lhs->field)) {
          throw std::runtime_error(
                  "Attempted operation '" + lhs->field + op->field + rhs->field +
                  "' on uninitialized variable '" + lhs->field + "'.");
        }
        lhs->value.setShared(mVariables[lhs->field]);
      }
      if (rhs->type == VARIABLE) {
        if (!isVariable(rhs->field)) {
          throw std::runtime_error(
                  "Attempted operation '" + lhs->field + op->field + rhs->field +
                  "' on uninitialized variable '" + rhs->field + "'.");
        }
        rhs->value.setShared(mVariables[rhs->field]);
      }
      if (rhs->value.matrix().size() == 1) {
        if (lhs->value.isLocal()) {
          if (op->field == "+" || op->field == ".+") {
            lhs->value.local().array() += rhs->value.matrix()(0, 0);
          } else {                                 // if(op->field == "-" || op->field == ".-")
            lhs->value.local().array() -= rhs->value.matrix()(0, 0);
          }
        } else {
          if (op->field == "+" || op->field == ".+") {
            lhs->value.local() = lhs->value.matrix().array() + rhs->value.matrix()(0, 0);
          } else {                                 // if(op->field == "-" || op->field == ".-")
            lhs->value.local() = lhs->value.matrix().array() - rhs->value.matrix()(0, 0);
          }
          lhs->value.mapLocal();
          lhs->type = VALUE;
        }
      } else if (lhs->value.matrix().size() == 1) {
        typename Derived::Scalar temp = lhs->value.matrix()(0, 0);
        if (op->field == "+" || op->field == ".+") {
          lhs->value.local() = rhs->value.matrix().array() + temp;
        } else {                           // if(op->field == "-" || op->field == ".-")
          lhs->value.local() = Derived::Constant(
            rhs->value.matrix().rows(),
            rhs->value.matrix().cols(), temp).array() - rhs->value.matrix().array();
        }
        lhs->value.mapLocal();
        lhs->type = VALUE;
      } else if (  // NOLINT
        lhs->value.matrix().rows() == rhs->value.matrix().rows() &&
        lhs->value.matrix().cols() == rhs->value.matrix().cols())
      {
        if (lhs->value.isLocal()) {
          if (op->field == "+" || op->field == ".+") {
            lhs->value.local().array() += rhs->value.matrix().array();
          } else {                                 // if(op->field == "-" || op->field == ".-")
            lhs->value.local().array() -= rhs->value.matrix().array();
          }
        } else {
          if (op->field == "+" || op->field == ".+") {
            lhs->value.local() = lhs->value.matrix().array() + rhs->value.matrix().array();
          } else {                                 // if(op->field == "-" || op->field == ".-")
            lhs->value.local() = lhs->value.matrix().array() - rhs->value.matrix().array();
          }
          lhs->value.mapLocal();
          lhs->type = VALUE;
        }
      } else {
        throw std::runtime_error(
                "Invalid operand dimensions for operation '" +
                lhs->field + op->field + rhs->field + "'.");
      }
      chunks.erase(op, rhs + 1);
      op = lhs + 1;
      rhs = (op != chunks.end()) ? op + 1 : op;
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
      operationPerformed = true;
#       endif
#endif
    } else {
      lhs = op;
      op = rhs;
      rhs++;
    }
  }
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  if (operationPerformed) {std::cout << "+: "; printChunks(chunks); std::cout << std::endl;}
#       endif
#endif
}

template<typename Derived>
void Parser<Derived>::evalAssignment(ChunkArray & chunks)
{
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  bool operationPerformed = false;
#       endif
#endif
  if (chunks.size() < 3) {return;}
  typename ChunkArray::iterator rhs = chunks.end() - 1, op = rhs - 1, lhs = op - 1;
  for (; op != chunks.begin() && rhs != chunks.begin(); ) {
    if (op->type == OPERATOR && op->field == "=" &&
      (lhs->type == VALUE || lhs->type == VARIABLE) &&
      (rhs->type == VALUE || rhs->type == VARIABLE))
    {
      if (rhs->type == VARIABLE) {
        if (!isVariable(rhs->field)) {
          throw std::runtime_error(
                  "Attempted operation '" + lhs->field + op->field +
                  rhs->field + "' on uninitialized variable '" + rhs->field + "'.");
        }
        rhs->value.setShared(mVariables[rhs->field]);
      }
      if (lhs->type == VALUE) {
        lhs->value.local() = rhs->value.matrix();
        lhs->value.mapLocal();
      } else {    // if(lhs->type == VARIABLE) {
        if (isVariable(lhs->field)) {
          lhs->value.setShared(mVariables[lhs->field]);
          if (lhs->row0 == -1) {
            if (lhs->value.matrix().rows() == rhs->value.matrix().rows() &&
              lhs->value.matrix().cols() == rhs->value.matrix().cols())
            {
              lhs->value.matrix() = rhs->value.matrix();
            } else {
              mVariables[lhs->field].local() = rhs->value.matrix();
              mVariables[lhs->field].mapLocal();
            }
          } else {    // if(lhs->row0 != -1) {
            lhs->value.matrix().block(
              lhs->row0, lhs->col0, lhs->rows,
              lhs->cols) = rhs->value.matrix();
          }
        } else {
          mVariables[lhs->field].local() = rhs->value.matrix();
          mVariables[lhs->field].mapLocal();
        }
      }
      rhs = chunks.erase(op, rhs + 1);
      op = (rhs != chunks.begin()) ? rhs - 1 : rhs;
      if (op != chunks.begin()) {lhs = op - 1;}
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
      operationPerformed = true;
#       endif
#endif
    } else {
      rhs = op;
      op = lhs;
      lhs--;
    }
  }
#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
  if (operationPerformed) {std::cout << "=: "; printChunks(chunks); std::cout << std::endl;}
#       endif
#endif
}

#ifdef DEBUG
#       ifdef EIGENLAB_DEBUG
template<typename Derived>
void Parser<Derived>::printChunks(
  ChunkArray & chunks, size_t maxRows, size_t maxCols,
  int precision)
{
  std::cout << "__";
  for (typename ChunkArray::iterator it = chunks.begin(); it != chunks.end(); it++) {
    switch (it->type) {
      case VALUE:
        std::cout << textRepresentation(it->value, maxRows, maxCols, precision);
        if (it->row0 != -1) {
          std::cout << "(" << it->row0 << ":" << it->row0 + it->rows - 1 << "," << it->col0 <<
            ":" << it->col0 + it->cols - 1 << ")";
        }
        break;
      case VARIABLE:
        std::cout << it->field;
        // << "=" << textRepresentation(mVariables[it->field], maxRows, maxCols, precision);
        if (it->row0 != -1) {
          std::cout << "(" << it->row0 << ":" << it->row0 + it->rows - 1 << "," << it->col0 <<
            ":" << it->col0 + it->cols - 1 << ")";
        }
        break;
      case OPERATOR:
        std::cout << it->field;
        break;
      case FUNCTION:
        std::cout << "f()=" << it->field;
        break;
    }
    std::cout << "__";
  }
}

template<typename Derived>
void Parser<Derived>::printVars(size_t maxRows, size_t maxCols, int precision)
{
  for (typename ValueMap::iterator it = mVariables.begin(); it != mVariables.end(); it++) {
    std::cout << it->first << " (" << it->second.matrix().rows() << "x" <<
      it->second.matrix().cols() << ") = " << textRepresentation(
      it->second, maxRows, maxCols,
      precision) << std::endl;
  }
}

template<typename Derived>
std::string Parser<Derived>::textRepresentation(
  Value<Derived> & val, size_t maxRows,
  size_t maxCols, int precision)
{
  if (val.matrix().size() == 1) {
    return numberToString<typename Derived::Scalar>(val.matrix()(0, 0), precision);
  } else {
    std::string str = "[";
    for (size_t row = 0; row < val.matrix().rows() && row < maxRows; row++) {
      str += (row > 0 ? ";[" : "[");
      for (size_t col = 0; col < val.matrix().cols() && col < maxCols; col++) {
        if (col > 0) {str += ",";}
        str += numberToString<typename Derived::Scalar>(val.matrix()(row, col), precision);
      }
      str += (val.matrix().cols() > maxCols ? "...]" : "]");
    }
    str += (val.matrix().rows() > maxRows ? "...]" : "]");
    return str;
  }
}
#       endif  // #ifdef EIGENLAB_DEBUG
#endif  // #ifdef DEBUG

template<typename Derived>
std::string Parser<Derived>::trim(const std::string & str)
{
  if (str.empty()) {return str;}
  std::string::const_iterator first = str.begin();
  std::string::const_iterator last = str.end() - 1;
  while (first < last && isspace(*first)) {first++;}
  while (first < last && isspace(*last)) {last--;}
  return std::string(first, last + 1);
}

template<typename Derived>
std::vector<std::string> Parser<Derived>::split(
  const std::string & str,
  const char delimeter)
{
  std::vector<std::string> args;
  std::string::const_iterator i0 = str.begin();
  for (std::string::const_iterator it = str.begin(); it != str.end(); it++) {
    if ((*it) == delimeter) {
      args.push_back(trim(std::string(i0, it)));
      i0 = it + 1;
    }
  }
  args.push_back(std::string(i0, str.end()));
  return args;
}

template<typename Derived>
template<typename T>
bool Parser<Derived>::isNumber(const std::string & str, T * num)
{
  std::istringstream iss(str);
  if (num) {
    iss >> (*num);
  } else {
    T number;
    iss >> number;
  }
  return !iss.fail() && !iss.bad() && iss.eof();
}

template<typename Derived>
template<typename T>
T Parser<Derived>::stringToNumber(const std::string & str)
{
  std::istringstream iss(str);
  T number;
  iss >> number;
  if (iss.fail() || iss.bad() || !iss.eof()) {
    throw std::runtime_error("Failed to convert " + str + " to a number.");
  }
  return number;
}

template<typename Derived>
template<typename T>
std::string Parser<Derived>::numberToString(T num, int precision)
{
  std::ostringstream oss;
  if (precision) {
    oss << std::setprecision(precision) << num;
  } else {
    oss << num;
  }
  return oss.str();
}

#ifdef DEBUG
template<typename Derived>
void
Parser<Derived>::test_w_lt(
  size_t & numFails,
  typename Derived::Scalar & /* s */,
  Derived & a34,
  Derived & b34,
  Derived & /* c43 */,
  Derived & /* v */, std::true_type)
{
  //
  // tests that only work if Derived::Scalar has operator<
  //
  Value<Derived> resultValue;
  Derived resultMatrix;
  Derived temp;
  std::cout << "Test min(a): ";
  resultValue = eval("min(a)");
  resultMatrix.setConstant(1, 1, a34.minCoeff());
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test min(a, 0): ";
  resultValue = eval("min(a, 0)");
  resultMatrix = a34.colwise().minCoeff();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test min(a, 1): ";
  resultValue = eval("min(a, 1)");
  resultMatrix = a34.rowwise().minCoeff();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test max(a): ";
  resultValue = eval("max(a)");
  resultMatrix.setConstant(1, 1, a34.maxCoeff());
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test max(a, 0): ";
  resultValue = eval("max(a, 0)");
  resultMatrix = a34.colwise().maxCoeff();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test max(a, 1): ";
  resultValue = eval("max(a, 1)");
  resultMatrix = a34.rowwise().maxCoeff();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test absmax(a): ";
  resultValue = eval("absmax(a)");
  resultMatrix.setConstant(
    1, 1, std::abs(a34.maxCoeff()) >= std::abs(
      a34.minCoeff()) ? a34.maxCoeff() : a34.minCoeff());
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test absmax(a, 0): ";
  resultValue = eval("absmax(a, 0)");
  resultMatrix = a34.colwise().maxCoeff();
  temp = a34.colwise().minCoeff();
  for (Index i = 0; i < resultMatrix.size(); ++i) {
    if (std::abs(resultMatrix(i)) < std::abs(temp(i))) {
      resultMatrix(i) = temp(i);
    }
  }
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test absmax(a, 1): ";
  resultValue = eval("absmax(a, 1)");
  resultMatrix = a34.rowwise().maxCoeff();
  temp = a34.rowwise().minCoeff();
  for (Index i = 0; i < resultMatrix.size(); ++i) {
    if (std::abs(resultMatrix(i)) < std::abs(temp(i))) {
      resultMatrix(i) = temp(i);
    }
  }
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test cwiseMin(a, b): ";
  resultValue = eval("cwiseMin(a, b)");
  resultMatrix = a34.cwiseMin(b34);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test cwiseMax(a, b): ";
  resultValue = eval("cwiseMax(a, b)");
  resultMatrix = a34.cwiseMax(b34);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }
}

template<typename Derived>
void
Parser<Derived>::test_w_lt(
  size_t & /* numFails */,
  typename Derived::Scalar & /* s */,
  Derived & /* a34 */,
  Derived & /* b34 */,
  Derived & /* c43 */,
  Derived & /* v */, std::false_type)
{
  // do nothing
}

template<typename Derived>
size_t Parser<Derived>::test()
{
  std::cout << std::endl;
  std::cout << "BEGIN unit test for EigenLab..." << std::endl;
  std::cout << "Make sure this function completes successfuly and prints the message "
    "'Successfully completed unit test for EigenLab with no failures.'" << std::endl;
  std::cout << std::endl;

  size_t numFails = 0;
  Value<Derived> resultValue;
  Derived resultMatrix;
  Derived temp;
  typename Derived::Scalar s = 2;

  Derived a34 = Derived::Random(3, 4);
  Derived b34 = Derived::Random(3, 4);
  Derived c43 = Derived::Random(4, 3);
  Derived v = Derived::Random(1, 10);
  // std::cout << "a34=" << std::endl << a34 << std::endl << std::endl;
  // std::cout << "b34=" << std::endl << b34 << std::endl << std::endl;
  // std::cout << "c43=" << std::endl << c43 << std::endl << std::endl;
  // std::cout << "v=" << std::endl << v << std::endl << std::endl;

  var("a").setShared(a34);
  var("b").setShared(b34);
  var("c").setShared(c43);
  var("v").setShared(v);
  var("s").setShared(&s);

  ////////////////////////////////////////
  std::cout << "Testing basic operations..." << std::endl << std::endl;
  ////////////////////////////////////////

  std::cout << "Test matrix addition a + b: ";
  resultValue = eval("a + b");
  resultMatrix = a34 + b34;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix/scalar addition a + s: ";
  resultValue = eval("a + s");
  resultMatrix = a34.array() + s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test scalar/matrix addition s + b: ";
  resultValue = eval("s + b");
  resultMatrix = b34.array() + s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix addition a .+ b: ";
  resultValue = eval("a .+ b");
  resultMatrix = a34 + b34;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix/scalar addition a .+ s: ";
  resultValue = eval("a .+ s");
  resultMatrix = a34.array() + s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test scalar/matrix addition s .+ b: ";
  resultValue = eval("s .+ b");
  resultMatrix = b34.array() + s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix subtraction a - b: ";
  resultValue = eval("a - b");
  resultMatrix = a34 - b34;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix/scalar subtraction a - s: ";
  resultValue = eval("a - s");
  resultMatrix = a34.array() - s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test scalar/matrix subtraction s - b: ";
  resultValue = eval("s - b");
  resultMatrix = (-b34.array()) + s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix subtraction a .- b: ";
  resultValue = eval("a .- b");
  resultMatrix = a34 - b34;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix/scalar subtraction a .- s: ";
  resultValue = eval("a .- s");
  resultMatrix = a34.array() - s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test scalar/matrix subtraction s .- b: ";
  resultValue = eval("s .- b");
  resultMatrix = (-b34.array()) + s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix negation -a: ";
  resultValue = eval("-a");
  resultMatrix = -a34;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test scalar negation -s: ";
  resultValue = eval("-s");
  resultMatrix.setConstant(1, 1, -s);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix coefficient-wise multiplication a .* b: ";
  resultValue = eval("a .* b");
  resultMatrix = a34.array() * b34.array();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix/scalar coefficient-wise multiplication a * s: ";
  resultValue = eval("a * s");
  resultMatrix = a34.array() * s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test scalar/matrix coefficient-wise multiplication s * b: ";
  resultValue = eval("s * b");
  resultMatrix = b34.array() * s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix/scalar coefficient-wise multiplication a .* s: ";
  resultValue = eval("a .* s");
  resultMatrix = a34.array() * s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test scalar/matrix coefficient-wise multiplication s .* b: ";
  resultValue = eval("s .* b");
  resultMatrix = b34.array() * s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix coefficient-wise division a ./ b: ";
  resultValue = eval("a ./ b");
  resultMatrix = a34.array() / b34.array();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix/scalar coefficient-wise division a / s: ";
  resultValue = eval("a / s");
  resultMatrix = a34.array() / s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test scalar/matrix coefficient-wise division s / b: ";
  resultValue = eval("s / b");
  resultMatrix = Derived::Constant(b34.rows(), b34.cols(), s).array() / b34.array();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix/scalar coefficient-wise division a ./ s: ";
  resultValue = eval("a ./ s");
  resultMatrix = a34.array() / s;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test scalar/matrix coefficient-wise division s ./ b: ";
  resultValue = eval("s ./ b");
  resultMatrix = Derived::Constant(b34.rows(), b34.cols(), s).array() / b34.array();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix coefficient-wise power a .^ b: ";
  resultValue = eval("abs(a) .^ b");
  resultMatrix = a34;
  for (Index i = 0; i < a34.size(); ++i) {
    resultMatrix(i) = pow(std::abs(a34(i)), b34(i));
  }
  // std::cout << std::endl;
  // std::cout << "a=" << std::endl << a34 << std::endl << std::endl;
  // std::cout << "b=" << std::endl << b34 << std::endl << std::endl;
  // std::cout << "val=" << std::endl << resultValue.matrix() << std::endl << std::endl;
  // std::cout << "mat=" << std::endl << resultMatrix << std::endl << std::endl;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix/scalar coefficient-wise power a ^ s: ";
  resultValue = eval("abs(a) ^ s");
  resultMatrix = a34;
  for (Index i = 0; i < a34.size(); ++i) {
    resultMatrix(i) = pow(std::abs(a34(i)), s);
  }
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test scalar/matrix coefficient-wise power s ^ b: ";
  resultValue = eval("s ^ b");
  resultMatrix = b34;
  for (Index i = 0; i < b34.size(); ++i) {
    resultMatrix(i) = pow(s, b34(i));
  }
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix/scalar coefficient-wise power a .^ s: ";
  resultValue = eval("abs(a) .^ s");
  resultMatrix = a34;
  for (Index i = 0; i < a34.size(); ++i) {
    resultMatrix(i) = pow(std::abs(a34(i)), s);
  }
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test scalar/matrix coefficient-wise power s .^ b: ";
  resultValue = eval("s .^ b");
  resultMatrix = b34;
  for (Index i = 0; i < b34.size(); ++i) {
    resultMatrix(i) = pow(s, b34(i));
  }
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix multiplication a * b: ";
  resultValue = eval("a * c");
  resultMatrix = a34 * c43;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test grouped subexpression (a + b) * c: ";
  resultValue = eval("(a + b) * c");
  resultMatrix = (a34 + b34) * c43;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test nested groups ((a + (b * 3 + 1)) * c).^2: ";
  resultValue = eval("((a + (b * 3 + 1)) * c).^2");
  resultMatrix = ((a34.array() + (b34.array() * 3 + 1)).matrix() * c43).array().pow(2);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  ////////////////////////////////////////
  std::cout << std::endl << "Testing coefficient and submatrix block access..." << std::endl <<
    std::endl;
  ////////////////////////////////////////

  std::cout << "Test matrix coefficient access a(i,j): ";
  resultValue = eval("a(1,2)");
  resultMatrix.setConstant(1, 1, a34(1, 2));
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test submatrix block access a(i:p,j:q): ";
  resultValue = eval("a(1:2,2:3)");
  resultMatrix = a34.block(1, 2, 2, 2);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test submatrix block access using 'end' and ':' identifiers a(i:end,:): ";
  resultValue = eval("a(1:end,:)");
  resultMatrix = a34.block(1, 0, a34.rows() - 1, a34.cols());
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test submatrix block access using subexpressions: ";
  resultValue = eval("a(2-1:2-1,0+1:3-1)");
  resultMatrix = a34.block(1, 1, 1, 2);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test submatrix block access using subexpressions with 'end' keyword: ";
  resultValue = eval("a(2-1:end-1,0+1:end-1)");
  resultMatrix = a34.block(1, 1, a34.rows() - 2, a34.cols() - 2);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test vector coefficient access v(i): ";
  resultValue = eval("v(5)");
  resultMatrix.setConstant(1, 1, v(5));
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test subvector segment access v(i:j): ";
  resultValue = eval("v(3:6)");
  resultMatrix = v.block(0, 3, 1, 4);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test subvector segment access using 'end' identifier v(i:end): ";
  resultValue = eval("v(5:end)");
  resultMatrix = v.block(0, 5, 1, v.cols() - 5);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test subvector segment access using ':' identifier v(:): ";
  resultValue = eval("v(:)");
  resultMatrix = v;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test subvector segment access using subexpressions: ";
  resultValue = eval("v(3-1:5+2)");
  resultMatrix = v.block(0, 2, 1, 6);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test subvector segment access using subexpressions with 'end' keyword: ";
  resultValue = eval("v((end-8)*2:end-3)");
  resultMatrix = v.block(0, 2, 1, 5);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  ////////////////////////////////////////
  std::cout << std::endl << "Testing vector/matrix expressions..." << std::endl << std::endl;
  ////////////////////////////////////////

  std::cout << "Test numeric range expression [i:j]: ";
  resultValue = eval("[2:5]");
  resultMatrix.resize(1, 4);
  resultMatrix << 2, 3, 4, 5;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test numeric range expression [i:s:j]: ";
  resultValue = eval("[2:2:10]");
  resultMatrix.resize(1, 5);
  resultMatrix << 2, 4, 6, 8, 10;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test numeric range with subexpressions: ";
  resultValue = eval("[6-2:5*2-3]");
  std::cout << "val=" << std::endl << resultValue.matrix() << std::endl << std::endl;
  resultMatrix.resize(1, 4);
  resultMatrix << 4, 5, 6, 7;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix expression [[1, 2]; [3, 4]]: ";
  resultValue = eval("[[1, 2]; [3, 4]]");
  resultMatrix.resize(2, 2);
  resultMatrix << 1, 2, 3, 4;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test matrix expression [  1, 2, 3;  4:6 ]: ";
  resultValue = eval("[1, 2, 3; 4:6]");
  resultMatrix.resize(2, 3);
  resultMatrix << 1, 2, 3, 4, 5, 6;
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  ////////////////////////////////////////
  std::cout << std::endl << "Testing coefficient-wise functions..." << std::endl << std::endl;
  ////////////////////////////////////////

  std::cout << "Test coefficient-wise abs(a): ";
  resultValue = eval("abs(a)");
  resultMatrix.resize(3, 4);
  resultMatrix.real() = a34.array().abs();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test coefficient-wise sqrt(a): ";
  resultValue = eval("sqrt(abs(a))");
  resultMatrix.real() = a34.array().abs().sqrt();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test coefficient-wise exp(a): ";
  resultValue = eval("exp(abs(a) + 0.001)");
  resultMatrix.real() = (a34.array().abs() + 0.001).exp();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test coefficient-wise log(a): ";
  resultValue = eval("log(abs(a) + 0.001)");
  resultMatrix.real() = (a34.array().abs() + 0.001).log();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test coefficient-wise log10(a): ";
  resultValue = eval("log10(abs(a) + 0.001)");
  resultMatrix.real() = (a34.array().abs() + 0.001).log() * (1.0 / log(10));
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test coefficient-wise sin(a): ";
  resultValue = eval("sin(a)");
  resultMatrix = a34.array().sin();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test coefficient-wise cos(a): ";
  resultValue = eval("cos(a)");
  resultMatrix = a34.array().cos();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test coefficient-wise tan(a): ";
  resultValue = eval("tan(a)");
  resultMatrix = a34.array().tan();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test coefficient-wise asin(a): ";
  resultValue = eval("asin(a)");
  resultMatrix = a34.array().asin();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test coefficient-wise acos(a): ";
  resultValue = eval("acos(a)");
  resultMatrix = a34.array().acos();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  ////////////////////////////////////////
  std::cout << std::endl << "Testing matrix reduction functions..." << std::endl << std::endl;
  ////////////////////////////////////////

  std::cout << "Test trace(a): ";
  resultValue = eval("trace(a)");
  resultMatrix.setConstant(1, 1, a34.trace());
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test norm(a): ";
  resultValue = eval("norm(a)");
  resultMatrix.setConstant(1, 1, a34.norm());
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test size(a, 0): ";
  resultValue = eval("size(a, 0)");
  resultMatrix.setConstant(1, 1, a34.rows());
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test size(a, 1): ";
  resultValue = eval("size(a, 1)");
  resultMatrix.setConstant(1, 1, a34.cols());
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  test_w_lt(numFails, s, a34, b34, c43, v, has_operator_lt<typename Derived::Scalar>());

  std::cout << "Test mean(a): ";
  resultValue = eval("mean(a)");
  resultMatrix.setConstant(1, 1, a34.mean());
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test mean(a, 0): ";
  resultValue = eval("mean(a, 0)");
  resultMatrix = a34.colwise().mean();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test mean(a, 1): ";
  resultValue = eval("mean(a, 1)");
  resultMatrix = a34.rowwise().mean();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test sum(a): ";
  resultValue = eval("sum(a)");
  resultMatrix.setConstant(1, 1, a34.sum());
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test sum(a, 0): ";
  resultValue = eval("sum(a, 0)");
  resultMatrix = a34.colwise().sum();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test sum(a, 1): ";
  resultValue = eval("sum(a, 1)");
  resultMatrix = a34.rowwise().sum();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test prod(a): ";
  resultValue = eval("prod(a)");
  resultMatrix.setConstant(1, 1, a34.prod());
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test prod(a, 0): ";
  resultValue = eval("prod(a, 0)");
  resultMatrix = a34.colwise().prod();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test prod(a, 1): ";
  resultValue = eval("prod(a, 1)");
  resultMatrix = a34.rowwise().prod();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  ////////////////////////////////////////
  std::cout << std::endl << "Testing matrix functions..." << std::endl << std::endl;
  ////////////////////////////////////////

  std::cout << "Test transpose(a): ";
  resultValue = eval("transpose(a)");
  resultMatrix = a34.transpose();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test conjugate(a): ";
  resultValue = eval("conjugate(a)");
  resultMatrix = a34.conjugate();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test adjoint(a): ";
  resultValue = eval("adjoint(a)");
  resultMatrix = a34.adjoint();
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  ////////////////////////////////////////
  std::cout << std::endl << "Testing matrix initializers..." << std::endl << std::endl;
  ////////////////////////////////////////

  std::cout << "Test zeros(5): ";
  resultValue = eval("zeros(5)");
  resultMatrix = Derived::Zero(5, 5);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test ones(5): ";
  resultValue = eval("ones(5)");
  resultMatrix = Derived::Ones(5, 5);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test eye(5): ";
  resultValue = eval("eye(5)");
  resultMatrix = Derived::Identity(5, 5);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  try {
    std::cout << "Test zeros(5.2): ";
    resultValue = eval("zeros(5.2)");    // <-- Should NOT succeed!!!
    std::cout << "FAIL" << std::endl; ++numFails;
  } catch (std::runtime_error & err) {
    std::cout << err.what() << std::endl;
    std::cout << "Exception caught, so we're OK" << std::endl;
  }

  try {
    std::cout << "Test eye(-3): ";
    resultValue = eval("eye(-3)");    // <-- Should NOT succeed!!!
    std::cout << "FAIL" << std::endl; ++numFails;
  } catch (std::runtime_error & err) {
    std::cout << err.what() << std::endl;
    std::cout << "Exception caught, so we're OK" << std::endl;
  }

  std::cout << "Test zeros(4,7): ";
  resultValue = eval("zeros(4,7)");
  resultMatrix = Derived::Zero(4, 7);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test ones(4,7): ";
  resultValue = eval("ones(4,7)");
  resultMatrix = Derived::Ones(4, 7);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test eye(4,7): ";
  resultValue = eval("eye(4,7)");
  resultMatrix = Derived::Identity(4, 7);
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  ////////////////////////////////////////
  std::cout << std::endl << "Testing variable assignment..." << std::endl << std::endl;
  ////////////////////////////////////////

  std::cout << "Test assigning to a variable with the same dimensions a = b: ";
  eval("a = b");
  if (a34.isApprox(b34)) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test assigning to a variable with different dimensions a = c: ";
  eval("a = c");
  if (var("a").matrix().isApprox(c43) && a34.isApprox(b34)) {
    std::cout << "OK" << std::endl;
  } else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }
  var("a").setShared(a34);

  std::cout << "Test creating a new variable x = [1,2;3,4]: ";
  resultValue = eval("x = [1,2;3,4]");
  if (var("x").matrix().isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test assigning to a variable coefficient a(i,j) = s: ";
  eval("a(1, 2) = s");
  if (a34(1, 2) == s) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test assigning to a variable submatrix block a(0:1,1:2) = x: ";
  resultValue = eval("a(0:1,1:2) = x");
  if (a34.block(0, 1, 2, 2).isApprox(var("x").matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  try {
    std::cout << "Test bad function call: ";
    resultValue = eval("foobar(-3)");    // <-- Should NOT succeed!!!
    std::cout << "FAIL" << std::endl; ++numFails;
  } catch (std::runtime_error & err) {
    std::cout << err.what() << std::endl;
    std::cout << "Exception caught, so we're OK" << std::endl;
  }
  ////////////////////////////////////////
  std::cout << std::endl << "Testing fp parsing..." << std::endl << std::endl;
  ////////////////////////////////////////

  std::cout << "Test assigning 1.2e-3: ";
  resultValue = eval("s = 1.2e-3");
  resultMatrix.setConstant(1, 1, typename Derived::Scalar(1.2e-3));
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test assigning 1.e3: ";
  resultValue = eval("s = 1.e3");
  resultMatrix.setConstant(1, 1, typename Derived::Scalar(1000));
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << "Test assigning 12.34e05: ";
  resultValue = eval("s = 12.34e05");
  resultMatrix.setConstant(1, 1, typename Derived::Scalar(123.4e4));
  if (resultMatrix.isApprox(resultValue.matrix())) {std::cout << "OK" << std::endl;} else {
    std::cout << "FAIL" << std::endl; ++numFails;
  }

  std::cout << std::endl;
  if (numFails == 0) {
    std::cout << "Successfully completed unit test for EigenLab with no failures." << std::endl;
  } else {
    std::cout << "Completed unit test for EigenLab with " << numFails <<
      " failures (see above)." << std::endl;
  }
  std::cout << std::endl;
  return numFails;
}
#endif  // DEBUG

}  // namespace EigenLab
#endif  // EIGENLAB__EIGENLAB_HPP_
