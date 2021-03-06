#include <list>
#include <moveit/task_constructor/cost_queue.h>
#include <gtest/gtest.h>

template<typename ValueType, typename CostType>
std::ostream& operator<<(std::ostream &os, const cost_ordered<ValueType, CostType> &queue) {
	for (const auto &pair : queue.sorted())
		os << pair.cost() << ": " << pair.value() << std::endl;
	return os;
}

template <typename ValueType, typename CostType = int>
class CostOrderedTest : public ::testing::Test {
protected:
	typedef std::deque<ValueType> container_type;
	typedef cost_ordered<ValueType, std::deque<ValueType>, CostType> queue_type;

	CostOrderedTest() {}

	auto insert(ValueType value, CostType cost = CostType()) {
		return queue.insert(value, cost);
	}

	void fill(int first=1, int last=5) {
		for (; first < last; ++first)
			insert(first, first);
	}

	cost_ordered<ValueType, CostType> queue;

	void SetUp() {}
	void TearDown() {}
};
typedef CostOrderedTest<int, int> CostOrderedTestInt;

TEST_F(CostOrderedTestInt, ordered_push) {
	auto top = *insert(2,2);
	for (int i = 3; i < 6; ++i) {
		insert(i, i);
		EXPECT_EQ(queue.top(), top);
	}
}

TEST_F(CostOrderedTestInt, reverse_ordered_push) {
	for (int i = 6; i > 3; --i) {
		insert(i, i);
		EXPECT_EQ(queue.top().cost(), i);
		EXPECT_EQ(queue.top().value(), i);
	}
}

TEST_F(CostOrderedTestInt, update) {
	fill(3, 5);
	auto first = queue.top();
	auto added = *insert(7, 3);
	// items with same cost are added behind existing values
	EXPECT_EQ(queue.top(), first);
	EXPECT_EQ(*(++queue.begin()), added);
}
