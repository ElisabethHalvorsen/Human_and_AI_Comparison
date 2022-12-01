import carla
from pandas import DataFrame


def remove_non_junctions(df: DataFrame, skip: int) -> DataFrame:
    rows_to_drop: list = []
    for i in range(df.index.stop):
        if (i - 1) % skip != 0:
            rows_to_drop.append(i)
    return df.drop(labels=rows_to_drop)


def remove_non_representable_scenarios(df: DataFrame, labels: list) -> DataFrame:
    return df.drop(
        labels=labels,
        axis=1,
        level=0)


def remove_redundant_rows(df: DataFrame, labels: list, skip: int) -> DataFrame:
    df = remove_non_junctions(df, skip)
    return remove_non_representable_scenarios(df, labels)


def merge_two_and_more_vehicle_crashes(df: DataFrame) -> DataFrame:
    iteration = 0
    for i in df.columns[1:]:
        if iteration % 3 == 0:
            weather = i[0]
            try:
                df[weather, 'Two-Vehicle Crash'] = df[weather]['Two-Vehicle Crash'] + df[weather][
                    'More Than Two-Vehicle']
            except KeyError:  # if there is no 'More Than Two-Vehicle' column
                pass
        iteration += 1
    df = df.rename(columns={'Two-Vehicle Crash': 'Multiple Vehicles'},
              level=1)  # Update Column name
    df = df.drop('More Than Two-Vehicle', axis=1, level=1)  # Remove redundant col name
    return df


def print_to_html(df: DataFrame, file_name: str):
    df.to_html(f'outputs/{file_name}.html')


if __name__ == '__main__':
    client = carla.Client('localhost', 2000)
    world = client.get_world()
#%%
